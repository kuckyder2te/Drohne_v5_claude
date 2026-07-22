// Eigenstaendiges Tool (frueher TEST_MOTORS in config.h/TestModes.cpp).
// Bauen/Flashen: pio run -e test_motors --target upload && pio device monitor
//
// Befehle kommen ueber BT_UART (Serial1/HC-06), nicht ueber USB-Serial -
// das entspricht dem bisherigen Verhalten (Motor-Rohbefehle liefen schon
// immer direkt ueber Serial1, unabhaengig von COMM_USE_BLUETOOTH).
#include <Arduino.h>
#include "myLogger.h"
#include "config.h"
#include "pins.h"
#include "MotorMixer.h"
#include "Battery.h"

// Minimale dlog()-Implementierung: schreibt direkt auf Serial, ohne
// CommChannel/PIDController/Settings zu benoetigen (die braucht nur die
// normale Firmware in src/myLogger.cpp).
void dlog(const String &msg)
{
    Serial.println(msg);
}

MotorMixer motors;
Battery battery;
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

void setup()
{
    Serial.begin(115200);
    Serial1.setTX(PIN_BT_TX);
    Serial1.setRX(PIN_BT_RX);
    Serial1.begin(BT_BAUD);
    delay(2000);

    LOG(">> Modus: MOTORTEST");
    printMotorHelp();
    motors.begin();
    battery.begin();
}

void loop()
{
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
