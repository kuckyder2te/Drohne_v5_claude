// Eigenstaendiges Tool (frueher TEST_MOTORS_SINGLE in config.h/TestModes.cpp).
// Bauen/Flashen: pio run -e test_motors_single --target upload && pio device monitor
//
// Befehle kommen ueber BT_UART (Serial1/HC-06), nicht ueber USB-Serial -
// das entspricht dem bisherigen Verhalten (Motor-Rohbefehle liefen schon
// immer direkt ueber Serial1, unabhaengig von CLI_USE_BLUETOOTH).
#include <Arduino.h>
#include "myLogger.h"
#include "config.h"
#include "pins.h"
#include "MotorMixer.h"
#include "Battery.h"

// Formatierpuffer der *_FMT-Makros. src/myLogger.cpp wird in dieser
// Umgebung nicht mitkompiliert (build_src_filter), die Definition muss
// also hier stehen - auch lib/ nutzt die Makros. Groesse muss zu der
// Deklaration in include/myLogger.h passen.
char logBuf[160];

MotorMixer motors;
Battery battery;
uint16_t singleThrottle = ESC_MIN_US;
uint8_t activeMotor = 0;

void setup()
{
    Serial.begin(115200);

    // Vorgabe der Bibliothek ist WARNING - ohne diese Zeile bliebe jede
    // LOGGER_NOTICE-Ausgabe unsichtbar. Ausgabe laeuft ueber
    // Logger::defaultLog nach Serial, eine eigene Ausgabefunktion
    // braucht das Tool nicht.
    Logger::setLogLevel(Logger::NOTICE);
    Serial1.setTX(PIN_BT_TX);
    Serial1.setRX(PIN_BT_RX);
    Serial1.begin(BT_BAUD);
    delay(2000);

    LOGGER_NOTICE(">> Modus: EINZELMOTOR TEST");
    LOGGER_NOTICE("1=FL 2=FR 3=BR 4=BL + - s");
    motors.begin();
    battery.begin();
}

void loop()
{
    static uint32_t lastBatS = 0;
    if (millis() - lastBatS >= 5000)
    {
        lastBatS = millis();
        LOGGER_NOTICE_FMT("[BAT] %.2fV", battery.getVoltage());
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
            LOGGER_NOTICE("[MOTOR] FL aktiv");
            break;
        case '2':
            activeMotor = 2;
            singleThrottle = ESC_MIN_US;
            motors.stop();
            LOGGER_NOTICE("[MOTOR] FR aktiv");
            break;
        case '3':
            activeMotor = 3;
            singleThrottle = ESC_MIN_US;
            motors.stop();
            LOGGER_NOTICE("[MOTOR] BR aktiv");
            break;
        case '4':
            activeMotor = 4;
            singleThrottle = ESC_MIN_US;
            motors.stop();
            LOGGER_NOTICE("[MOTOR] BL aktiv");
            break;
        case '+':
            if (activeMotor > 0)
            {
                singleThrottle = constrain(singleThrottle + THROTTLE_STEP, ESC_MIN_US, ESC_MAX_US);
                motors.setSingle(activeMotor, singleThrottle);
                LOGGER_NOTICE_FMT("[MOTOR] Throttle: %d us", singleThrottle);
            }
            break;
        case '-':
            if (activeMotor > 0)
            {
                singleThrottle = constrain(singleThrottle - THROTTLE_STEP, ESC_MIN_US, ESC_MAX_US);
                motors.setSingle(activeMotor, singleThrottle);
                LOGGER_NOTICE_FMT("[MOTOR] Throttle: %d us", singleThrottle);
            }
            break;
        case 's':
        case 'S':
            activeMotor = 0;
            singleThrottle = ESC_MIN_US;
            motors.stop();
            LOGGER_NOTICE("[MOTOR] STOP");
            break;
        }
    }
}
