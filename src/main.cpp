#include <Arduino.h>
#include "myLogger.h"
#include "config.h"
#include "pins.h"
#include "control/MotorMixer.h"
#include "control/PIDController.h"
#include "sensor/Barometer.h"
#include "comm/KeyboardInput.h"
#include "comm/BluetoothConfig.h"
#include "storage/Settings.h"
#include "sensor/IMU.h"

MotorMixer motors;
Barometer baro;
KeyboardInput keyboard;
PIDController pidHeight(PID_KP_HEIGHT, PID_KI_HEIGHT, PID_KD_HEIGHT, true); // mit Offset
PIDController pidRoll(PID_KP_ROLL, PID_KI_ROLL, PID_KD_ROLL, false);        // ohne Offset
PIDController pidPitch(PID_KP_PITCH, PID_KI_PITCH, PID_KD_PITCH, false);    // ohne Offset

BluetoothConfig btConfig;
Settings settings;
IMU imu;

// ── Zustandsvariablen ──────────────────────────────────────
float targetHeightCm = 0.0f;
bool armed = false;
uint32_t lastPidMs = 0;
uint32_t lastPrintMs = 0;

// ── Test-Modi ──────────────────────────────────────────────
#ifdef TEST_MOTORS
uint16_t currentThrottle = ESC_MIN_US;
void printMotorHelp()
{
    LOG("─────────────────────────────");
    LOG(" MOTORTEST: + - s h");
    LOG("─────────────────────────────");
}
#endif

// ── Hilfsfunktionen ────────────────────────────────────────
void i2cScan()
{
    LOG("[I2C] Scanne Bus...");
    int found = 0;
    for (uint8_t addr = 1; addr < 127; addr++)
    {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0)
        {
            LOG_FMT("[I2C] Gefunden: 0x%02X", addr);
            found++;
        }
    }
    if (found == 0)
        LOG("[I2C] Kein Geraet!");
    LOG_FMT("[I2C] Gesamt: %d Geraet(e)", found);
}

void printHelp()
{
    LOG("────────────────────────────────────");
    LOG(" Pfeil hoch   = Zielhoehe +10 cm");
    LOG(" Pfeil runter = Zielhoehe -10 cm");
    LOG(" a = ARM (Motoren ein)");
    LOG(" s = DISARM (Motoren aus)");
    LOG(" r = Barometer rekalibrieren");
    LOG(" h = Hilfe");
    LOG("────────────────────────────────────");
}

void disarm()
{
    armed = false;
    targetHeightCm = 0.0f;
    motors.stop();
    pidHeight.reset();
    pidRoll.reset();  // ← NEU
    pidPitch.reset(); // ← NEU
    LOG("[CTRL] DISARM — Motoren gestoppt");
}

// ── Setup ──────────────────────────────────────────────────
void setup()
{
    Serial.begin(115200);
    delay(2000);

    // BT zuerst starten — damit LOG() auf BT ausgeben kann
    btConfig.begin();

    pidHeight.begin();
    pidRoll.begin();
    pidPitch.begin();

    // Jetzt erst LOG verwenden
    LOG("=== DROHNE PICO BOOT ====");

#ifdef TEST_I2C_SCAN
    Wire.setSDA(PIN_SDA);
    Wire.setSCL(PIN_SCL);
    Wire.begin();
    LOG("Wire OK, scanne...");
    i2cScan();
#endif
    /*
    #ifdef TEST_IMU
        LOG(">> Modus: IMU TEST");
        Wire.setSDA(PIN_SDA);
        Wire.setSCL(PIN_SCL);
        Wire.begin();
        Wire.setClock(400000);  // ← explizit 400kHz setzen
        delay(500);              // ← war 100ms, jetzt 500ms
        if (!imu.begin()) {
            LOG("FEHLER: IMU! Programm gestoppt.");
            while (true) delay(1000);
        }
    #endif
    */

#ifdef TEST_IMU
    LOG(">> Modus: IMU TEST");

    delay(1000);

    if (!imu.begin(true)) // ← true = Wire initialisieren
    {
        LOG("FEHLER: IMU! Programm gestoppt.");
        while (true)
            delay(1000);
    }
#endif

#ifdef TEST_MOTORS
    LOG(">> Modus: MOTORTEST");
    printMotorHelp();
    motors.begin();
#endif

#ifdef TEST_BAROMETER
    LOG(">> Modus: BAROMETER TEST");
    if (!baro.begin())
    {
        LOG("FEHLER: Barometer nicht gefunden.");
        while (true)
            delay(1000);
    }
#endif

#ifdef TEST_KEYBOARD
    LOG(">> Modus: KEYBOARD TEST");
    if (!baro.begin())
    {
        LOG("FEHLER: Barometer nicht gefunden.");
        while (true)
            delay(1000);
    }
    keyboard.begin();
    printHelp();
#endif

#ifndef TEST_MOTORS
#ifndef TEST_BAROMETER
#ifndef TEST_KEYBOARD
#ifndef TEST_IMU
#ifndef TEST_I2C_SCAN
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

    motors.begin();
    pidHeight.begin();
    keyboard.begin();

    settings.begin();
    float kp, ki, kd;
    if (settings.load(kp, ki, kd))
    {
        pidHeight.setKp(kp);
        pidHeight.setKi(ki);
        pidHeight.setKd(kd);
    }

    // IMU immer starten — unabhaengig von EEPROM
    if (!imu.begin(false))
    {
        LOG("WARNUNG: IMU nicht gefunden!");
    }

    printHelp();
    LOG("[CTRL] Bereit — 'a' zum Armen");
#endif // TEST_I2C_SCAN
#endif // TEST_IMU
#endif // TEST_KEYBOARD
#endif // TEST_BAROMETER
#endif // TEST_MOTORS
}

// ── Loop ───────────────────────────────────────────────────
void loop()
{
    // ── TEST_IMU ──────────────────────────────────────────
#ifdef TEST_IMU
    imu.update();
    static uint32_t lastIMU = 0;
    if (millis() - lastIMU >= 100)
    {
        lastIMU = millis();
        LOG_FMT("[IMU] Roll: %.1f  Pitch: %.1f  AccZ: %.2f  ready:%d",
                imu.getRoll(), imu.getPitch(), imu.getAccZ(), imu.isReady());
    }
#endif

    // ── TEST_MOTORS ────────────────────────────────────────
#ifdef TEST_MOTORS
    if (Serial.available())
    {
        char cmd = Serial.read();
        switch (cmd)
        {
        case '+':
            currentThrottle += THROTTLE_STEP;
            motors.setThrottle(currentThrottle);
            break;
        case '-':
            currentThrottle -= THROTTLE_STEP;
            motors.setThrottle(currentThrottle);
            break;
        case 's':
        case 'S':
            currentThrottle = ESC_MIN_US;
            motors.stop();
            break;
        case 'h':
        case 'H':
            printMotorHelp();
            break;
        }
    }
#endif

    // ── TEST_BAROMETER ─────────────────────────────────────
#ifdef TEST_BAROMETER
    baro.update();
    static uint32_t lastBaro = 0;
    if (millis() - lastBaro >= 500)
    {
        lastBaro = millis();
        LOG_FMT("[BARO] Hoehe: %.1f cm | Druck: %.2f hPa | Temp: %.1f C",
                baro.getAltitudeCm(),
                baro.getPressure(),
                baro.getTemperature());
    }
#endif

    // ── TEST_KEYBOARD ──────────────────────────────────────
#ifdef TEST_KEYBOARD
    baro.update();
    KeyEvent key = keyboard.getKey();
    switch (key)
    {
    case KeyEvent::ARROW_UP:
        LOG("[KEY] Pfeil HOCH");
        break;
    case KeyEvent::ARROW_DOWN:
        LOG("[KEY] Pfeil RUNTER");
        break;
    case KeyEvent::KEY_S:
        LOG("[KEY] STOP");
        break;
    case KeyEvent::KEY_R:
        baro.calibrate();
        break;
    case KeyEvent::KEY_H:
        printHelp();
        break;
    case KeyEvent::KEY_A:
        LOG("[KEY] ARM — nur im Normalbetrieb");
        break;
    case KeyEvent::NONE:
        break;
    }
    static uint32_t lastPrint = 0;
    if (millis() - lastPrint >= 500)
    {
        lastPrint = millis();
        LOG_FMT("[BARO] Hoehe: %.1f cm", baro.getAltitudeCm());
    }
#endif

    // ── NORMALBETRIEB ──────────────────────────────────────
#ifndef TEST_MOTORS
#ifndef TEST_BAROMETER
#ifndef TEST_KEYBOARD
#ifndef TEST_IMU
#ifndef TEST_I2C_SCAN

    baro.update();
    imu.update();

    // Tastatureingabe ZUERST
    KeyEvent key = keyboard.getKey();
    // Dann BT PID-Konfiguration
    btConfig.update(pidHeight, pidRoll, pidPitch, settings);

    switch (key)
    {
    case KeyEvent::ARROW_UP:
        targetHeightCm = constrain(targetHeightCm + 10.0f, THROTTLE_MIN_CM, MAX_HEIGHT_CM);
        LOG_FMT("[CTRL] Zielhoehe: %.1f cm", targetHeightCm);
        break;

    case KeyEvent::ARROW_DOWN:
        targetHeightCm = constrain(targetHeightCm - 10.0f, THROTTLE_MIN_CM, MAX_HEIGHT_CM);
        LOG_FMT("[CTRL] Zielhoehe: %.1f cm", targetHeightCm);
        break;

    case KeyEvent::KEY_A:
        if (!armed)
        {
            LOG("[CTRL] Rekalibrierung vor ARM...");
            baro.calibrate();
            delay(500);
            armed = true;
            targetHeightCm = 20.0f;
            pidHeight.reset();
            pidRoll.reset();  // ← NEU
            pidPitch.reset(); // ← NEU
            LOG("[CTRL] ARM — Ziel: 20 cm");
        }
        break;

    case KeyEvent::KEY_S:
        disarm();
        break;

    case KeyEvent::KEY_R:
    if (!armed) {          // ← nur wenn NICHT armed!
        baro.calibrate();
        pidHeight.reset();
    } else {
        LOG("[CTRL] Rekalibrierung nur im DISARM Modus!");
    }
    break;

    case KeyEvent::KEY_H:
        printHelp();
        break;

    case KeyEvent::NONE:
        break;
    }

    // PID-Regelkreis
    if (armed && (millis() - lastPidMs >= PID_INTERVAL_MS))
    {
        lastPidMs = millis();

        // Hoehenregelung
        float throttle = pidHeight.compute(targetHeightCm, baro.getAltitudeCm());

        // Lageregelung
        float rollCorr = pidRoll.compute(TARGET_ROLL_DEG, imu.getRoll());
        float pitchCorr = pidPitch.compute(TARGET_PITCH_DEG, imu.getPitch());

        // Alles kombinieren
        motors.mix((uint16_t)throttle, rollCorr, pitchCorr, 0.0f);
    }

    // Statusausgabe alle 500ms
    if (millis() - lastPrintMs >= 500)
    {
        lastPrintMs = millis();
        LOG_FMT("[CTRL] Ziel: %.1f cm | Ist: %.1f cm | Throttle: %.0f us | Armed: %s",
                targetHeightCm,
                baro.getAltitudeCm(),
                pidHeight.getLastThrottle(),
                armed ? "JA" : "NEIN");
    }

#endif // TEST_I2C_SCAN
#endif // TEST_IMU
#endif // TEST_KEYBOARD
#endif // TEST_BAROMETER
#endif // TEST_MOTORS
}