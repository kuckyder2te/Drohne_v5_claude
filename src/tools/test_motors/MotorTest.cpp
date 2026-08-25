// Eigenstaendiges Tool (frueher TEST_MOTORS in config.h/TestModes.cpp).
// Bauen/Flashen: pio run -e test_motors --target upload && pio device monitor
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
uint16_t currentThrottle = ESC_MIN_US;

void printMotorHelp()
{
    LOGGER_NOTICE("-----------------------------------------");
    LOGGER_NOTICE(" MOTORTEST: + - s h");
    LOGGER_NOTICE(" p = ESC-Strom EIN/AUS (MOSFET GP28)");
    LOGGER_NOTICE(" c = Kalibrierung: ESC-Strom AUS");
    LOGGER_NOTICE(" k = MAX senden + ESC-Strom EIN");
    LOGGER_NOTICE(" m = MIN senden (Kalibrierung fertig)");
    LOGGER_NOTICE("-----------------------------------------");
}

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

    LOGGER_NOTICE(">> Modus: MOTORTEST");
    printMotorHelp();
    // Bewusst stromlos starten: der ESC leitet seine Betriebsart aus dem
    // Signal ab, das beim Einschalten anliegt. Erst 'p' (Normalbetrieb, MIN
    // liegt an) oder 'k' (Kalibrierung, MAX liegt an) legt ihn fest.
    motors.begin(false);
    battery.begin();
    LOGGER_NOTICE("[ESC] Strom AUS - 'p' fuer Normalbetrieb, 'c' fuer Kalibrierung");
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
        LOGGER_NOTICE_FMT("[BAT] %.2fV", battery.getVoltage());
    }

    char cmd = 0;
    static uint32_t lastByteMillis = 0;
    if (BT_UART.available())
    {
        cmd = BT_UART.read();
        uint32_t now = millis();
        // Ein Mensch tippt keine 20 ms auseinander - schnellere Bytes sind
        // Leitungsrauschen (z.B. beim Trennen der BT-Verbindung), keine
        // echten Kommandos.
        if (now - lastByteMillis < 20)
            cmd = 0;
        lastByteMillis = now;
    }

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
        case 'p':
        case 'P':
            currentThrottle = ESC_MIN_US;
            motors.stop();
            if (motors.isPowered())
            {
                motors.powerOff();
                LOGGER_NOTICE("[ESC] Strom AUS");
            }
            else
            {
                // Immer erst MIN ausgeben, dann einschalten - sonst startet
                // der ESC in der Kalibrierung statt im Normalbetrieb.
                delay(ESC_PWM_SETTLE_MS);
                motors.powerOn();
                LOGGER_NOTICE("[ESC] Strom EIN bei MIN - Normalbetrieb");
            }
            break;
        case 'c':
        case 'C':
            LOGGER_NOTICE("[ESC] SCHRITT 1: ESC-Strom wird abgeschaltet (MOSFET GP28)");
            LOGGER_NOTICE("[ESC] Dann 'k' druecken - Pico sendet MAX und schaltet ein");
            currentThrottle = ESC_MIN_US;
            motors.stop();
            motors.powerOff();
            break;
        case 'k':
        case 'K':
            if (motors.isPowered())
            {
                // Ein bereits laufender ESC geht durch nachtraegliches MAX
                // nicht in die Kalibrierung - die liest er nur beim Hochlaufen.
                LOGGER_NOTICE("[ESC] ESCs sind am Strom - erst 'c' druecken!");
                break;
            }
            LOGGER_NOTICE("[ESC] SCHRITT 2: MAX (2000us) liegt an, schalte ESC-Strom EIN");
            currentThrottle = ESC_MAX_US;
            motors.setThrottle(currentThrottle);
            delay(ESC_PWM_SETTLE_MS);
            motors.powerOn();
            LOGGER_NOTICE("[ESC] Warte auf ESC-Piepstoene, dann 'm' druecken");
            break;
        case 'm':
        case 'M':
            LOGGER_NOTICE("[ESC] SCHRITT 3: Sende MIN (1000us)...");
            currentThrottle = ESC_MIN_US;
            motors.setThrottle(currentThrottle);
            LOGGER_NOTICE("[ESC] Kalibrierung abgeschlossen (2x Pieps = OK)");
            break;
        }
    }
}
