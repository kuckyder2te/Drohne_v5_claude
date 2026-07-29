// Eigenstaendiges Tool (frueher TEST_IMU in config.h/TestModes.cpp).
// Bauen/Flashen: pio run -e test_imu --target upload && pio device monitor
#include <Arduino.h>
#include "myLogger.h"
#include "config.h"
#include "pins.h"
#include "IMU.h"

// Formatierpuffer der *_FMT-Makros. src/myLogger.cpp wird in dieser
// Umgebung nicht mitkompiliert (build_src_filter), die Definition muss
// also hier stehen - auch lib/ nutzt die Makros. Groesse muss zu der
// Deklaration in include/myLogger.h passen.
char logBuf[160];

IMU imu;

void setup()
{
    Serial.begin(115200);

    // Vorgabe der Bibliothek ist WARNING - ohne diese Zeile bliebe jede
    // LOGGER_NOTICE-Ausgabe unsichtbar. Ausgabe laeuft ueber
    // Logger::defaultLog nach Serial, eine eigene Ausgabefunktion
    // braucht das Tool nicht.
    Logger::setLogLevel(Logger::NOTICE);
    delay(2000);
    LOGGER_NOTICE(">> Modus: IMU TEST");

    delay(1000);

    if (!imu.begin(true))
    {
        LOGGER_NOTICE("FEHLER: IMU! Programm gestoppt.");
        while (true)
            delay(1000);
    }
}

void loop()
{
    imu.update();
    static uint32_t lastIMU = 0;
    if (millis() - lastIMU >= 100)
    {
        lastIMU = millis();
        LOGGER_NOTICE_FMT("[IMU] Roll: %.1f  Pitch: %.1f  AccZ: %.2f  ready:%d",
                imu.getRoll(), imu.getPitch(), imu.getAccZ(), imu.isReady());
    }
}
