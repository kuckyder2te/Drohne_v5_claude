#include <Arduino.h>
#include "myLogger.h"
#include "config.h"
#include "pins.h"
#include "control/MotorMixer.h"
#include "control/PIDController.h"
#include "sensor/Barometer.h"
#include "comm/CommChannel.h"
#include "comm/cli.h"
#include "storage/Settings.h"
#include "sensor/IMU.h"
#include "sensor/Battery.h"
#include "sensor/Ultrasonic.h"
#include "testmode/TestModes.h"
#include "control/InputHandler.h"

MotorMixer motors;
Barometer baro;
Battery battery;
Ultrasonic ultrasonic;
PIDController pidHeight(PID_KP_HEIGHT, PID_KI_HEIGHT, PID_KD_HEIGHT, true); // mit Offset
PIDController pidRoll(PID_KP_ROLL, PID_KI_ROLL, PID_KD_ROLL, false);        // ohne Offset
PIDController pidPitch(PID_KP_PITCH, PID_KI_PITCH, PID_KD_PITCH, false);    // ohne Offset

CommChannel* comm = nullptr;   // aktiver Eingabe-/Log-Kanal, Auswahl per COMM_USE_BLUETOOTH (config.h)
Settings settings;
IMU imu;

// -- Zustandsvariablen --------------------------------------
float targetHeightCm = 0.0f;
bool armed = false;
bool statusLogEnabled = false;
bool armPending = false;
uint32_t armPendingMs = 0;
uint32_t lastPidMs = 0;
uint32_t lastPrintMs = 0;

// -- Hilfsfunktionen ----------------------------------------
void printHelp()
{
    LOG("------------------------------------");
    LOG(" a   = ARM (2x bestaetigen)");
    LOG(" d   = DISARM (sofort)");
    LOG(" +/- = Hoehe +/-10 cm (sofort)");
    LOG(" r   = Baro rekalibrieren");
    LOG(" l   = Statuslog ein/aus");
    LOG(" h   = Hilfe");
    LOG(" ?   = PID-Werte anzeigen");
    LOG(" PID: P=x.x  I=x.x  D=x.x");
    LOG(" SAVE / RESET");
    LOG("------------------------------------");
}

void disarm()
{
    armed = false;
    targetHeightCm = 0.0f;
    motors.stop();
    pidHeight.reset();
    pidRoll.reset();
    pidPitch.reset();
    LOG("[CTRL] DISARM - Motoren gestoppt");
}


// -- Setup --------------------------------------------------
void setup()
{
#ifdef COMM_USE_BLUETOOTH
    Serial1.setTX(PIN_BT_TX);
    Serial1.setRX(PIN_BT_RX);
    Serial1.begin(BT_BAUD);
    comm = new CommChannel(Serial1);
    
#else
    Serial.begin(115200);
    comm = new CommChannel(Serial);
    cli::begin(Serial);
#endif
    Serial.begin(115200);
    cli::begin(Serial);

  delay(2000);

    comm->sendLine("[BT] Drohne bereit");
    comm->sendLine("[BT] Befehle: A D R L H  +/-");
    comm->sendLine("[BT] PID: P=x I=x D=x  RP= RI= RD=  PP= PI= PD=");
    comm->sendLine("[BT] S=Speichern  RESET  ?=Abfrage");
    comm->sendLine("[CLI] Neu: ':' + Zeile fuer Shell-Befehle, z.B. :setHeight 30  (:help fuer Liste)");
    LOG("[BT] Bluetooth bereit");

    battery.begin();
    ultrasonic.begin();

    pidHeight.begin();
    pidRoll.begin();
    pidPitch.begin();

    LOG("=== DROHNE PICO BOOT ====");

    TestModes::setup();

#ifdef NORMALBETRIEB
    LOG(">> Modus: NORMALBETRIEB");

    Wire.setSDA(PIN_SDA);
    Wire.setSCL(PIN_SCL);
    Wire.begin();

    if (!baro.begin())
    {
        LOG("FEHLER: Barometer! Programm gestoppt.");
        while (true)
            delay(1000);
    }

    ultrasonic.begin();
    motors.begin();
    pidHeight.begin();

    settings.begin();
    float kp, ki, kd;
    if (settings.load(kp, ki, kd))
    {
        pidHeight.setKp(kp);
        pidHeight.setKi(ki);
        pidHeight.setKd(kd);
    }

    battery.begin();

    if (!imu.begin(false))
    {
        LOG("WARNUNG: IMU nicht gefunden!");
    }

    printHelp();
    LOG("[CTRL] Bereit - 'a' zum Armen");

#endif // NORMALBETRIEB
}

// -- Loop ---------------------------------------------------
void loop()
{
    cli::update();
    battery.update();

    TestModes::loop();

    // -- NORMALBETRIEB --------------------------------------
#ifdef NORMALBETRIEB

    baro.update();
    ultrasonic.update();
    imu.update();

    // Bei IMU Fehler oder Höhensprung → sofort DISARM
    if (armed)
    {
        if (!imu.isReady())
        {
            LOG("[SAFETY] IMU Fehler - DISARM!");
            disarm();
        }
        static float lastHeight = 0;
        float h = baro.getAltitudeCm();
        if (abs(h - lastHeight) > 500.0f)
        {
            LOG("[SAFETY] Hoehensprung - DISARM!");
            disarm();
        }
        lastHeight = h;
    }

    InputHandler::handle();

    // PID-Regelkreis
    if (armed && (millis() - lastPidMs >= PID_INTERVAL_MS))
    {
        lastPidMs = millis();

        bool airborne = ultrasonic.isValid() &&
                        (ultrasonic.getAltitudeCm() > LIFTOFF_HEIGHT_CM);
        pidHeight.enableIntegral(airborne);
        pidRoll.enableIntegral(airborne);
        pidPitch.enableIntegral(airborne);

        float currentHeight = ultrasonic.isValid() ? ultrasonic.getAltitudeCm() : baro.getAltitudeCm();
        float throttle = pidHeight.compute(targetHeightCm, currentHeight);

        float rollCorr = pidRoll.compute(TARGET_ROLL_DEG, imu.getRoll());
        float pitchCorr = pidPitch.compute(TARGET_PITCH_DEG, imu.getPitch());

        motors.mix((uint16_t)throttle, rollCorr, pitchCorr, 0.0f);
    }

    // Statusausgabe alle 500ms
    if (statusLogEnabled && (millis() - lastPrintMs >= 500))
    {
        lastPrintMs = millis();
        LOG_FMT("[CTRL] Ziel: %.1f cm | Ist: %.1f cm | Throttle: %.0f us | Armed: %s | Bat: %.2fV | Druck: %.2f hPa",
                targetHeightCm,
                ultrasonic.isValid() ? ultrasonic.getAltitudeCm() : baro.getAltitudeCm(),
                pidHeight.getLastThrottle(),
                armed ? "JA" : "NEIN",
                battery.getVoltage(),
                baro.getPressure());
    }

#endif // NORMALBETRIEB
}
