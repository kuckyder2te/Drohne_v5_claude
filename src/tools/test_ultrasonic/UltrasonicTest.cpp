// Eigenstaendiges Tool (frueher TEST_ULTRASONIC in config.h/TestModes.cpp).
// Bauen/Flashen: pio run -e test_ultrasonic --target upload && pio device monitor
#include <Arduino.h>
#include "myLogger.h"
#include "config.h"
#include "pins.h"
#include "Ultrasonic.h"

// Formatierpuffer der *_FMT-Makros. src/myLogger.cpp wird in dieser
// Umgebung nicht mitkompiliert (build_src_filter), die Definition muss
// also hier stehen - auch lib/ nutzt die Makros. Groesse muss zu der
// Deklaration in include/myLogger.h passen.
char logBuf[160];

Ultrasonic ultrasonic;

void setup()
{
    Serial.begin(115200);

    // Vorgabe der Bibliothek ist WARNING - ohne diese Zeile bliebe jede
    // LOGGER_NOTICE-Ausgabe unsichtbar. Ausgabe laeuft ueber
    // Logger::defaultLog nach Serial, eine eigene Ausgabefunktion
    // braucht das Tool nicht.
    Logger::setLogLevel(Logger::NOTICE);
    delay(2000);
    LOGGER_NOTICE(">> Modus: ULTRASCHALL TEST");
    ultrasonic.begin();
}

void loop()
{
    ultrasonic.update();
    static uint32_t lastUltra = 0;
    if (millis() - lastUltra >= 200)
    {
        lastUltra = millis();
        if (ultrasonic.isValid())
        {
            LOGGER_NOTICE_FMT("[ULTRA] Hoehe: %.1f cm", ultrasonic.getAltitudeCm());
        }
        else
        {
            LOGGER_NOTICE("[ULTRA] Kein Signal!");
        }
    }
}
