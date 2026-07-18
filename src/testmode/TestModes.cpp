#include "testmode/TestModes.h"

#include <Arduino.h>
#include "myLogger.h"
#include "config.h"
#include "pins.h"
#include "control/MotorMixer.h"
#include "control/PIDController.h"
#include "sensor/Barometer.h"
#include "comm/CommChannel.h"
#include "storage/Settings.h"
#include "sensor/IMU.h"
#include "sensor/Battery.h"
#include "sensor/Ultrasonic.h"

// Zugriff auf die main.cpp-Globalen wie in src/comm/cli.cpp
// (extern float targetHeightCm;) / src/myLogger.cpp ueber extern-Globale.
extern MotorMixer motors;
extern Barometer baro;
extern Battery battery;
extern Ultrasonic ultrasonic;
extern PIDController pidHeight;
extern PIDController pidRoll;
extern PIDController pidPitch;
extern CommChannel* comm;
extern Settings settings;
extern IMU imu;
void printHelp(); // bleibt in main.cpp (auch vom Normalbetrieb genutzt)

// -- TEST_MOTORS ----------------------------------------------
#ifdef TEST_MOTORS
namespace {
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
}
#endif

// -- TEST_I2C_SCAN ----------------------------------------------
#ifdef TEST_I2C_SCAN
namespace {
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
}
#endif

void TestModes::setup()
{
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
}

void TestModes::loop()
{
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
    KeyEvent key = comm->getKey();
    String tkCmd = comm->getCommand();
    if (tkCmd.length() > 0)
        comm->processCommand(tkCmd, pidHeight, pidRoll, pidPitch, settings);
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
}
