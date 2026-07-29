#include "mode/NormalMode.h"

#include <Arduino.h>
#include "myLogger.h"
#include "config.h"
#include "pins.h"
#include "control/FlightController.h"
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
    LOGGER_NOTICE(">> Modus: NORMALBETRIEB");

    Wire.setSDA(PIN_SDA);
    Wire.setSCL(PIN_SCL);
    Wire.begin();

    if (!baro.begin())
    {
        LOGGER_NOTICE("FEHLER: Barometer! Programm gestoppt.");
        while (true)
            delay(1000);
    }

    ultrasonic.begin();
    flightController.begin(settings);

    battery.begin();

    if (!imu.begin(false))
    {
        LOGGER_NOTICE("WARNUNG: IMU nicht gefunden!");
    }

    LOGGER_NOTICE("[CTRL] Bereit - 'arm' zum Armen");
}

void NormalMode::loop()
{
    cli::update();

    baro.update();
    ultrasonic.update();
    imu.update();
    battery.update();

    // Bei IMU Fehler oder Höhensprung → sofort DISARM
    flightController.checkSafety(imu.isReady(), baro.getAltitudeCm());

    // ARM-Bestaetigung verfaellt nach 3 s (lief vorher in InputHandler::handle())
    flightController.updateArmPendingTimeout();

    // PID-Regelkreis
    flightController.updateControlLoop(ultrasonic, baro, imu);

    // Statusausgabe alle 500ms
    flightController.logStatus(battery, baro, ultrasonic);
}
