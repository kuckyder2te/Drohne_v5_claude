// Eigenstaendiges Tool (frueher TEST_BAROMETER in config.h/TestModes.cpp).
// Bauen/Flashen: pio run -e test_barometer --target upload && pio device monitor
#include <Arduino.h>
#include <Wire.h>
#include "myLogger.h"
#include "config.h"
#include "pins.h"
#include "Barometer.h"
#include "Battery.h"

// Formatierpuffer der *_FMT-Makros. src/myLogger.cpp wird in dieser
// Umgebung nicht mitkompiliert (build_src_filter), die Definition muss
// also hier stehen - auch lib/ nutzt die Makros. Groesse muss zu der
// Deklaration in include/myLogger.h passen.
char logBuf[160];

Barometer baro;
Battery battery;

void setup()
{
    Serial.begin(115200);

    // Vorgabe der Bibliothek ist WARNING - ohne diese Zeile bliebe jede
    // LOGGER_NOTICE-Ausgabe unsichtbar. Ausgabe laeuft ueber
    // Logger::defaultLog nach Serial, eine eigene Ausgabefunktion
    // braucht das Tool nicht.
    Logger::setLogLevel(Logger::NOTICE);
    delay(2000);
    LOGGER_NOTICE(">> Modus: BAROMETER TEST");

    Wire.setSDA(PIN_SDA);
    Wire.setSCL(PIN_SCL);
    Wire.begin();

    if (!baro.begin())
    {
        LOGGER_NOTICE("FEHLER: Barometer nicht gefunden.");
        while (true)
            delay(1000);
    }
    battery.begin();
}

void loop()
{
    baro.update();
    battery.update();
    static uint32_t lastBaro = 0;
    if (millis() - lastBaro >= 500)
    {
        lastBaro = millis();
        LOGGER_NOTICE_FMT("[BARO] Hoehe: %.1f cm | Druck: %.2f hPa | Temp: %.1f C",
                baro.getAltitudeCm(),
                baro.getPressure(),
                baro.getTemperature());
        LOGGER_NOTICE_FMT("[BAT] Spannung: %.2fV", battery.getVoltage());
    }
}
