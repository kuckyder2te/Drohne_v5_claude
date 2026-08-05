#include "mode/NormalMode.h"

#include <Arduino.h>
#include "myLogger.h"
#include "config.h"
#include "pins.h"
#include "control/FlightController.h"
#include "control/AttitudeLoop.h"
#include "control/SharedState.h"
#include "Barometer.h"
#include "Ultrasonic.h"
#include "IMU.h"
#include "Battery.h"
#include "comm/cli.h"
#include "storage/Settings.h"

// Gemeinsame Firmware-Objekte. Hier definiert (frueher in main.cpp), da
// NormalMode die alleinige Kompositionswurzel ist; per extern genutzt von
// FlightController/cli.
Barometer baro;
Battery battery;
Ultrasonic ultrasonic;
FlightController flightController;
Settings settings;
IMU imu;

namespace {
    // Klartext zu shared::ABORT_*. Die Uebersetzung gehoert auf Kern 0:
    // Kern 1 darf nicht loggen (ein globaler logBuf fuer beide Kerne).
    const char *abortText(uint16_t code) {
        switch (code) {
            case shared::ABORT_ANGLE:    return "Winkelgrenze";
            case shared::ABORT_TIMEOUT:  return "Zeit abgelaufen";
            case shared::ABORT_ESTOP:    return "Not-Aus";
            case shared::ABORT_C0:       return "Kern 0 stumm";
            case shared::ABORT_DISARM:   return "DISARM";
            case shared::ABORT_NOSWITCH: return "keine Schwingung - h erhoehen";
            case shared::ABORT_IMU:      return "IMU Fehler";
            case shared::ABORT_DONE:     return "fertig";
            default:                     return "";
        }
    }
}

void NormalMode::setup()
{
    Serial.begin(115200);

    // BT-UART aufsetzen, wenn dort die Shell haengt oder hin geloggt wird.
#if defined(CLI_USE_BLUETOOTH) || defined(_BT_LOG)
    Serial1.setTX(PIN_BT_TX);
    Serial1.setRX(PIN_BT_RX);
    Serial1.begin(BT_BAUD);
#endif

    // Logger erst anmelden, wenn die Zielstreams stehen: localLogger() schreibt
    // je nach _SERIAL_LOG/_BT_LOG auf Serial und/oder BT_UART, eine noch nicht
    // begonnene UART wuerde ins Leere laufen.
    Logger::setOutputFunction(&localLogger);
    // Vorgabe der Bibliothek ist WARNING - ohne diese Zeile blieben alle
    // LOGGER_NOTICE-Ausgaben unsichtbar. Wert kommt per -D_DEBUG_ aus platformio.ini.
    Logger::setLogLevel(Logger::_DEBUG_);

    delay(2000);

    // Die Shell ist ein Singleton mit genau einem Stream - die Auswahl laeuft
    // ueber CLI_USE_BLUETOOTH (config.h). cli::begin() meldet sich selbst auf
    // dem gewaehlten Kanal, damit der auch dann nicht tot wirkt, wenn LOGGER_NOTICE()
    // per _SERIAL_LOG/_BT_LOG woandershin geht.
#ifdef CLI_USE_BLUETOOTH
    cli::begin(BT_UART);
#else
    cli::begin(Serial);
#endif

    LOGGER_NOTICE("=== DROHNE PICO BOOT ====");
    LOGGER_NOTICE(">> Modus: NORMALBETRIEB (Lageregelung auf Kern 1)");

    Wire.setSDA(PIN_SDA);
    Wire.setSCL(PIN_SCL);
    Wire.begin();

#ifdef BARO_ENABLED
    if (!baro.begin())
    {
        LOGGER_NOTICE("FEHLER: Barometer! Programm gestoppt.");
        while (true)
            delay(1000);
    }
#else
    // Barometer bewusst nicht angefasst: der Lageregelkreis auf Kern 1 haelt
    // den I2C-Bus exklusiv, siehe BARO_ENABLED in config.h. Hoehenquelle ist
    // damit ausschliesslich der Ultraschall.
    LOGGER_NOTICE("[BARO] abgeschaltet - Hoehe kommt vom Ultraschall");
#endif

    ultrasonic.begin();
    flightController.begin(settings);

    battery.begin();

    if (!imu.begin(false))
    {
        LOGGER_NOTICE("WARNUNG: IMU nicht gefunden!");
    }

    LOGGER_NOTICE("[CTRL] Bereit - 'arm' zum Armen");

    // Erst hier darf Kern 1 loslegen: Wire, ESCs und IMU stehen jetzt.
    shared::g_beatC0    = 1;
    shared::g_core0Ready = true;
}

// ── Kern 1 ─────────────────────────────────────────────────────────────
// setup1()/loop1() sind schwache Symbole im earlephilhower-Core; sind sie
// definiert, startet main() Kern 1 automatisch - ohne zusaetzliches
// Build-Flag. Kern 1 laeuft dabei VOR setup() von Kern 0 an, deshalb das
// Warten auf g_core0Ready.
void setup1()
{
    while (!shared::g_core0Ready) tight_loop_contents();
    attitude::begin(imu, flightController.getMotors());
}

void loop1()
{
    attitude::step();
}

void NormalMode::loop()
{
    cli::update();

    uint32_t now = millis();

    // Feste Kadenzen statt "jeden Durchlauf": ultrasonic.update() blockiert
    // 31-55 ms, baro.update() 70 ms. Vorher lief diese Schleife dadurch mit
    // ~9 Hz und PID_INTERVAL_MS war wirkungslos.
    static uint32_t lastUltra = 0;
    if (now - lastUltra >= ULTRA_UPDATE_MS) {
        lastUltra = now;
        ultrasonic.update();
    }

#ifdef BARO_ENABLED
    static uint32_t lastBaro = 0;
    if (now - lastBaro >= BARO_UPDATE_MS) {
        lastBaro = now;
        baro.update();
    }
#endif

    battery.update();

    // Telemetrie von Kern 1 abholen
    shared::CoreTlm t;
    shared::tlmRead(t);

    // ── Herzschlag von Kern 1 ueberwachen ──────────────────────────────
    // Steht Kern 1, laufen die Motoren mit dem zuletzt geschriebenen Wert
    // weiter - das ist der gefaehrlichste Zustand des ganzen Umbaus.
    static uint32_t seenBeat = 0;
    static uint32_t seenMs   = 0;
    static bool     c1Dead   = false;
    uint32_t b1 = shared::g_beatC1;
    if (b1 != seenBeat) {
        seenBeat = b1;
        seenMs   = now;
        c1Dead   = false;
    } else if (!c1Dead && seenMs && (now - seenMs) > ATTITUDE_WATCHDOG_MS) {
        c1Dead = true;
        shared::g_estop = true;
        flightController.getMotors().stopFast();
        LOGGER_NOTICE("[SAFETY] Kern 1 antwortet nicht - DISARM!");
        flightController.disarm();
    }

    // Abbruchmeldungen von Kern 1 in Klartext uebersetzen
    static uint16_t lastAbort = shared::ABORT_NONE;
    if (t.abortCode != lastAbort) {
        lastAbort = t.abortCode;
        if (t.abortCode != shared::ABORT_NONE)
            LOGGER_NOTICE_FMT("[BENCH] Lauf beendet: %s", abortText(t.abortCode));
    }

    // Bei IMU Fehler oder Höhensprung → sofort DISARM
    flightController.checkSafety(t.imuReady, ultrasonic.getAltitudeCm());

    // ARM-Bestaetigung verfaellt nach 3 s
    flightController.updateArmPendingTimeout();

    // Hoehenregler (Roll/Pitch laufen auf Kern 1)
    flightController.updateControlLoop(ultrasonic, baro);

    // Statusausgabe alle 500ms
    flightController.logStatus(battery, baro, ultrasonic);

    shared::g_beatC0 = shared::g_beatC0 + 1;
}
