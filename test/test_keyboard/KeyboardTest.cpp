// Eigenstaendiges Tool (frueher TEST_KEYBOARD in config.h/TestModes.cpp).
// Bauen/Flashen: pio test -e rpipico -f test_keyboard --without-testing && pio device monitor
//
// Prueft, ob ueber den konfigurierten Eingabekanal (COMM_USE_BLUETOOTH ->
// Serial1/HC-06, sonst USB-Serial) Tastendruecke ankommen, und gibt parallel
// die Barometer-Hoehe aus. Anders als der fruehere in-Firmware-Modus liest
// dieses Tool die Bytes direkt vom Stream, ohne CommChannel/PIDController/
// Settings (die liegen in src/ und werden fuer test/-Tools nicht kompiliert) -
// PID-Tuning-Befehle testet weiterhin der Normalbetrieb der Firmware.
#include <Arduino.h>
#include <Wire.h>
#include "myLogger.h"
#include "config.h"
#include "pins.h"
#include "Barometer.h"

// Minimale dlog()-Implementierung: schreibt direkt auf Serial, ohne
// CommChannel/PIDController/Settings zu benoetigen (die braucht nur die
// normale Firmware in src/myLogger.cpp).
void dlog(const String &msg)
{
    Serial.println(msg);
}

Barometer baro;

// Eingabekanal wie in der Firmware per COMM_USE_BLUETOOTH waehlen.
#ifdef COMM_USE_BLUETOOTH
Stream &keyIn = Serial1;
#else
Stream &keyIn = Serial;
#endif

void printKeyHelp()
{
    LOG("-----------------------------------------");
    LOG(" KEYBOARDTEST: sendet Tasten ueber den");
    LOG(" konfigurierten Kanal (BT-UART oder USB).");
    LOG(" + / -  = Pfeil HOCH / RUNTER");
    LOG(" a d l  = Tasten-Echo");
    LOG(" r      = Barometer rekalibrieren");
    LOG(" h      = diese Hilfe");
    LOG("-----------------------------------------");
}

void setup()
{
    Serial.begin(115200);
#ifdef COMM_USE_BLUETOOTH
    Serial1.setTX(PIN_BT_TX);
    Serial1.setRX(PIN_BT_RX);
    Serial1.begin(BT_BAUD);
#endif
    delay(2000);

    LOG(">> Modus: KEYBOARD TEST");

    Wire.setSDA(PIN_SDA);
    Wire.setSCL(PIN_SCL);
    Wire.begin();

    if (!baro.begin())
    {
        LOG("FEHLER: Barometer nicht gefunden.");
        while (true)
            delay(1000);
    }
    printKeyHelp();
}

void loop()
{
    baro.update();

    while (keyIn.available())
    {
        char c = keyIn.read();
        switch (c)
        {
        case '+':
            LOG("[KEY] Pfeil HOCH");
            break;
        case '-':
            LOG("[KEY] Pfeil RUNTER");
            break;
        case 'a':
        case 'A':
            LOG("[KEY] A (ARM - nur im Normalbetrieb)");
            break;
        case 'd':
        case 'D':
            LOG("[KEY] D (DISARM)");
            break;
        case 'l':
        case 'L':
            LOG("[KEY] L (Statuslog)");
            break;
        case 'r':
        case 'R':
            baro.calibrate();
            LOG("[KEY] R - Barometer rekalibriert");
            break;
        case 'h':
        case 'H':
            printKeyHelp();
            break;
        case '\r':
        case '\n':
            break;
        default:
            LOG_FMT("[KEY] '%c' (0x%02X)", c, (uint8_t)c);
            break;
        }
    }

    static uint32_t lastPrint = 0;
    if (millis() - lastPrint >= 500)
    {
        lastPrint = millis();
        LOG_FMT("[BARO] Hoehe: %.1f cm", baro.getAltitudeCm());
    }
}
