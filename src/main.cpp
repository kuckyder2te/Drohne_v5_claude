#include <Arduino.h>
#include "myLogger.h"
#include "config.h"
#include "pins.h"
#include "control/MotorMixer.h"
#include "control/PIDController.h"
#include "sensor/Barometer.h"
#include "comm/BluetoothComm.h"
#include "comm/SerialInput.h"
#include "storage/Settings.h"
#include "sensor/IMU.h"
#include "sensor/Battery.h"
#include "sensor/Ultrasonic.h"

MotorMixer motors;
Barometer baro;
Battery battery;
Ultrasonic ultrasonic;
PIDController pidHeight(PID_KP_HEIGHT, PID_KI_HEIGHT, PID_KD_HEIGHT, true); // mit Offset
PIDController pidRoll(PID_KP_ROLL, PID_KI_ROLL, PID_KD_ROLL, false);        // ohne Offset
PIDController pidPitch(PID_KP_PITCH, PID_KI_PITCH, PID_KD_PITCH, false);    // ohne Offset

BluetoothComm bt;
SerialInput serial;
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

// -- Test-Modi ----------------------------------------------
#ifdef TEST_MOTORS
uint16_t currentThrottle = ESC_MIN_US;
void printMotorHelp()
{
    LOG("-----------------------------------------");
    LOG(" MOTORTEST: + - s h");
    LOG(" c = Kalibrierung: LiPo ZUERST trennen");
    LOG(" k = MAX senden (dann LiPo anstecken)");
    LOG(" m = MIN senden (Kalibrierung fertig)");
    LOG("-----------------------------------------");
}
#endif

// -- Hilfsfunktionen ----------------------------------------
void i2cBusRecovery()
{
    Wire.end();             // I2C Peripheral freigeben → Pins als GPIO nutzbar
    delay(10);
    pinMode(PIN_SDA, OUTPUT);
    pinMode(PIN_SCL, OUTPUT);
    digitalWrite(PIN_SDA, HIGH);
    for (int i = 0; i < 9; i++)
    {
        digitalWrite(PIN_SCL, HIGH); delayMicroseconds(10);
        digitalWrite(PIN_SCL, LOW);  delayMicroseconds(10);
    }
    // STOP-Condition erzeugen
    digitalWrite(PIN_SDA, LOW);  delayMicroseconds(10);
    digitalWrite(PIN_SCL, HIGH); delayMicroseconds(10);
    digitalWrite(PIN_SDA, HIGH); delayMicroseconds(10);
    delay(20);
    Wire.setSDA(PIN_SDA);
    Wire.setSCL(PIN_SCL);
    Wire.begin();
    delay(50);
}

void i2cScan()
{
    i2cBusRecovery();

    // SDA-Pegel prüfen: LOW nach Recovery = Hardwareproblem
    Wire.end();
    delay(5);
    pinMode(PIN_SDA, INPUT_PULLUP);
    delay(5);
    bool sdaOk = digitalRead(PIN_SDA);
    Wire.setSDA(PIN_SDA);
    Wire.setSCL(PIN_SCL);
    Wire.begin();

    if (!sdaOk)
    {
        LOG("[I2C] FEHLER: SDA bleibt LOW nach Recovery!");
        LOG("[I2C] -> Kurzschluss, defektes Geraet oder fehlendes Pull-up?");
        LOG("[I2C] -> LiPo + USB trennen, 10s warten, neu starten.");
        return;
    }

    LOG("[I2C] Scanne Bus...");
    int found = 0;
    for (uint8_t addr = 1; addr < 127; addr++)
    {
        // Doppelprüfung: endTransmission + ein Byte lesen (echter ACK-Test)
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() != 0) continue;

        Wire.requestFrom((int)addr, 1);
        if (Wire.available() < 1) continue;
        Wire.read();

        LOG_FMT("[I2C] Gefunden: 0x%02X", addr);
        found++;
    }
    if (found == 0)
        LOG("[I2C] Kein Geraet!");
    LOG_FMT("[I2C] Gesamt: %d Geraet(e)", found);
}

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
    Serial.begin(115200);
    delay(2000);

    bt.begin();

    battery.begin();
    ultrasonic.begin();

    pidHeight.begin();
    pidRoll.begin();
    pidPitch.begin();

    LOG("=== DROHNE PICO BOOT ====");

#ifdef TEST_ULTRASONIC
    LOG(">> Modus: ULTRASCHALL TEST");
    ultrasonic.begin();
#endif

#ifdef TEST_I2C_SCAN
    LOG(">> Modus: I2C SCAN TEST (alle 5 s)");
#endif

#ifdef TEST_IMU
    LOG(">> Modus: IMU TEST");

    delay(1000);

    if (!imu.begin(true))
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

#ifdef TEST_MOTORS_SINGLE
    LOG(">> Modus: EINZELMOTOR TEST");
    LOG("1=FL 2=FR 3=BR 4=BL + - s");
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
    printHelp();
#endif

#ifndef TEST_MOTORS
#ifndef TEST_MOTORS_SINGLE
#ifndef TEST_BAROMETER
#ifndef TEST_KEYBOARD
#ifndef TEST_IMU
#ifndef TEST_I2C_SCAN
#ifndef TEST_ULTRASONIC
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
    bt.flushEcho();

#endif // TEST_ULTRASONIC
#endif // TEST_I2C_SCAN
#endif // TEST_IMU
#endif // TEST_KEYBOARD
#endif // TEST_BAROMETER
#endif // TEST_MOTORS_SINGLE
#endif // TEST_MOTORS
}

// -- Loop ---------------------------------------------------
void loop()
{
    battery.update();

// -- TEST_ULTRASONIC ------------------------------------
#ifdef TEST_ULTRASONIC
    ultrasonic.update();
    static uint32_t lastUltra = 0;
    if (millis() - lastUltra >= 200)
    {
        lastUltra = millis();
        if (ultrasonic.isValid())
        {
            LOG_FMT("[ULTRA] Hoehe: %.1f cm", ultrasonic.getAltitudeCm());
        }
        else
        {
            LOG("[ULTRA] Kein Signal!");
        }
    }
#endif

    // -- TEST_IMU ------------------------------------------
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

    // -- TEST_MOTORS ----------------------------------------
#ifdef TEST_MOTORS

    static uint32_t lastBat = 0;
    static bool firstRun = true;
    if (firstRun)
    {
        lastBat = millis();
        firstRun = false;
    }

    if (millis() - lastBat >= 5000)
    {
        lastBat = millis();
        LOG_FMT("[BAT] %.2fV", battery.getVoltage());
    }

    char cmd = 0;
    if (BT_UART.available())
        cmd = BT_UART.read();

    if (cmd != 0)
    {
        switch (cmd)
        {
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
            case 'c':
            case 'C':
                LOG("[ESC] SCHRITT 1: Jetzt LiPo TRENNEN!");
                LOG("[ESC] Dann 'k' druecken - Pico sendet danach MAX (2000us)");
                LOG("[ESC] Erst nach 'k': LiPo wieder anstecken");
                currentThrottle = ESC_MIN_US;
                motors.stop();
                break;
            case 'k':
            case 'K':
                LOG("[ESC] SCHRITT 2: Sende MAX (2000us) - jetzt LiPo anstecken!");
                currentThrottle = ESC_MAX_US;
                motors.setThrottle(currentThrottle);
                LOG("[ESC] Warte auf ESC-Piepstoene, dann 'm' druecken");
                break;
            case 'm':
            case 'M':
                LOG("[ESC] SCHRITT 3: Sende MIN (1000us)...");
                currentThrottle = ESC_MIN_US;
                motors.setThrottle(currentThrottle);
                LOG("[ESC] Kalibrierung abgeschlossen (2x Pieps = OK)");
                break;
            }
        }
    }
#endif

#ifdef TEST_MOTORS_SINGLE
    static uint16_t singleThrottle = ESC_MIN_US;
    static uint8_t activeMotor = 0;

    static uint32_t lastBatS = 0;
    if (millis() - lastBatS >= 5000)
    {
        lastBatS = millis();
        LOG_FMT("[BAT] %.2fV", battery.getVoltage());
    }

    char cmd = 0;
    if (BT_UART.available())
        cmd = BT_UART.read();

    if (cmd != 0)
    {
        switch (cmd)
        {
        case '1':
            activeMotor = 1;
            singleThrottle = ESC_MIN_US;
            motors.stop();
            LOG("[MOTOR] FL aktiv");
            break;
        case '2':
            activeMotor = 2;
            singleThrottle = ESC_MIN_US;
            motors.stop();
            LOG("[MOTOR] FR aktiv");
            break;
        case '3':
            activeMotor = 3;
            singleThrottle = ESC_MIN_US;
            motors.stop();
            LOG("[MOTOR] BR aktiv");
            break;
        case '4':
            activeMotor = 4;
            singleThrottle = ESC_MIN_US;
            motors.stop();
            LOG("[MOTOR] BL aktiv");
            break;
        case '+':
            if (activeMotor > 0)
            {
                singleThrottle = constrain(singleThrottle + THROTTLE_STEP, ESC_MIN_US, ESC_MAX_US);
                motors.setSingle(activeMotor, singleThrottle);
                LOG_FMT("[MOTOR] Throttle: %d us", singleThrottle);
            }
            break;
        case '-':
            if (activeMotor > 0)
            {
                singleThrottle = constrain(singleThrottle - THROTTLE_STEP, ESC_MIN_US, ESC_MAX_US);
                motors.setSingle(activeMotor, singleThrottle);
                LOG_FMT("[MOTOR] Throttle: %d us", singleThrottle);
            }
            break;
        case 's':
        case 'S':
            activeMotor = 0;
            singleThrottle = ESC_MIN_US;
            motors.stop();
            LOG("[MOTOR] STOP");
            break;
        }
    }
#endif

    // -- TEST_BAROMETER -------------------------------------
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
    LOG_FMT("[BAT] Spannung: %.2fV", battery.getVoltage());
#endif

    // -- TEST_I2C_SCAN --------------------------------------
#ifdef TEST_I2C_SCAN
    static uint32_t lastI2C = 0;
    if (millis() - lastI2C >= 5000)
    {
        lastI2C = millis();
        i2cScan();
    }
#endif

    // -- TEST_KEYBOARD --------------------------------------
#ifdef TEST_KEYBOARD
    baro.update();
    KeyEvent key = bt.getKey();
    switch (key)
    {
    case KeyEvent::ARROW_UP:
        LOG("[KEY] Pfeil HOCH");
        break;
    case KeyEvent::ARROW_DOWN:
        LOG("[KEY] Pfeil RUNTER");
        break;
    case KeyEvent::KEY_D:
        LOG("[KEY] DISARM");
        break;
    case KeyEvent::KEY_R:
        baro.calibrate();
        break;
    case KeyEvent::KEY_H:
        printHelp();
        break;
    case KeyEvent::KEY_A:
        LOG("[KEY] ARM - nur im Normalbetrieb");
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

    // -- NORMALBETRIEB --------------------------------------
#ifndef TEST_MOTORS
#ifndef TEST_MOTORS_SINGLE
#ifndef TEST_BAROMETER
#ifndef TEST_KEYBOARD
#ifndef TEST_IMU
#ifndef TEST_I2C_SCAN
#ifndef TEST_ULTRASONIC

    baro.update();
    ultrasonic.update();
    imu.update();

    // ARM-Bestätigung Timeout
    if (armPending && (millis() - armPendingMs > 3000))
    {
        armPending = false;
        LOG("[CTRL] ARM abgebrochen (Timeout)");
    }

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

    // BT ist primäre Eingabe, USB Serial nur als Backup
    KeyEvent key = bt.getKey();
    String btCmd = bt.getCommand();
    if (btCmd.length() > 0)
        bt.processCommand(btCmd, pidHeight, pidRoll, pidPitch, settings);

    if (key == KeyEvent::NONE && btCmd.length() == 0) {
        key = serial.getKey();
        String serCmd = serial.getCommand();
        if (serCmd.length() > 0)
            bt.processCommand(serCmd, pidHeight, pidRoll, pidPitch, settings);
    }

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
            if (!armPending)
            {
                armPending = true;
                armPendingMs = millis();
                LOG("[CTRL] ARM? Nochmal 'a' druecken (3s)");
            }
            else if (millis() - armPendingMs <= 3000)
            {
                armPending = false;
                if (!imu.isReady())
                {
                    LOG("[CTRL] ARM verweigert - IMU nicht bereit!");
                }
                else
                {
                    LOG("[CTRL] Rekalibrierung vor ARM...");
                    baro.calibrate();
                    delay(500);
                    armed = true;
                    targetHeightCm = 20.0f;
                    lastPidMs = millis();
                    pidHeight.reset();
                    pidRoll.reset();
                    pidPitch.reset();
                    LOG("[CTRL] ARM - Ziel: 20 cm");
                }
            }
            else
            {
                armPending = false;
                LOG("[CTRL] ARM abgebrochen (Timeout)");
            }
        }
        break;

    case KeyEvent::KEY_D:
        disarm();
        break;

    case KeyEvent::KEY_R:
        if (!armed)
        {
            baro.calibrate();
            pidHeight.reset();
            LOG("[CTRL] Barometer rekalibriert");
        }
        else
        {
            LOG("[CTRL] Rekalibrierung nur im DISARM Modus!");
        }
        break;

    case KeyEvent::KEY_H:
        printHelp();
        break;

    case KeyEvent::KEY_L:
        statusLogEnabled = !statusLogEnabled;
        LOG_FMT("[CTRL] Statusausgabe: %s", statusLogEnabled ? "EIN" : "AUS");
        break;

    case KeyEvent::NONE:
        break;
    }

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

#endif // TEST_ULTRASONIC
#endif // TEST_I2C_SCAN
#endif // TEST_IMU
#endif // TEST_KEYBOARD
#endif // TEST_BAROMETER
#endif // TEST_MOTORS_SINGLE
#endif // TEST_MOTORS
}
