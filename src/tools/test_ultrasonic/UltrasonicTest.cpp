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
    // Gleiche Kadenz wie NormalMode::loop() (ULTRA_UPDATE_MS). Jeder Aufruf
    // misst genau einen Sensor im Wechsel, ein einzelner Sensor kommt also
    // alle 100 ms dran. Der Treiber sperrt zu dichte Pings zusaetzlich selbst
    // (ULTRASONIC_MIN_PING_MS), frueher tat das ein delay(30) in update().
    static uint32_t lastPing = 0;
    if (millis() - lastPing >= ULTRA_UPDATE_MS)
    {
        lastPing = millis();
        ultrasonic.update();
    }

    static uint32_t lastPrint = 0;
    if (millis() - lastPrint >= 200)
    {
        lastPrint = millis();
        // Beide Sensoren einzeln, damit sich die Montage pruefen laesst:
        // Papier vor EINEN Sensor halten - nur dessen Wert darf springen.
        LOGGER_NOTICE_FMT("[ULTRA] S1: %.1f cm (%s) | S2: %.1f cm (%s) | Hoehe: %.1f cm (%s) | spread: %.1f",
                          ultrasonic.getAltitudeCm(0), ultrasonic.isValid(0) ? "ok" : "--",
                          ultrasonic.getAltitudeCm(1), ultrasonic.isValid(1) ? "ok" : "--",
                          ultrasonic.getAltitudeCm(),  ultrasonic.isValid()  ? "ok" : "--",
                          ultrasonic.getSpreadCm());
    }
}
