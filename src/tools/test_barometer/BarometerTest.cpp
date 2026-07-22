// Eigenstaendiges Tool (frueher TEST_BAROMETER in config.h/TestModes.cpp).
// Bauen/Flashen: pio run -e test_barometer --target upload && pio device monitor
#include <Arduino.h>
#include <Wire.h>
#include "myLogger.h"
#include "config.h"
#include "pins.h"
#include "Barometer.h"
#include "Battery.h"

// Minimale dlog()-Implementierung: schreibt direkt auf Serial, ohne
// CommChannel/PIDController/Settings zu benoetigen (die braucht nur die
// normale Firmware in src/myLogger.cpp).
void dlog(const String &msg)
{
    Serial.println(msg);
}

Barometer baro;
Battery battery;

void setup()
{
    Serial.begin(115200);
    delay(2000);
    LOG(">> Modus: BAROMETER TEST");

    Wire.setSDA(PIN_SDA);
    Wire.setSCL(PIN_SCL);
    Wire.begin();

    if (!baro.begin())
    {
        LOG("FEHLER: Barometer nicht gefunden.");
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
        LOG_FMT("[BARO] Hoehe: %.1f cm | Druck: %.2f hPa | Temp: %.1f C",
                baro.getAltitudeCm(),
                baro.getPressure(),
                baro.getTemperature());
        LOG_FMT("[BAT] Spannung: %.2fV", battery.getVoltage());
    }
}
