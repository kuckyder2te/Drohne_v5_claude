// Eigenstaendiges Tool (frueher TEST_ULTRASONIC in config.h/TestModes.cpp).
// Bauen/Flashen: pio run -e test_ultrasonic --target upload && pio device monitor
#include <Arduino.h>
#include "myLogger.h"
#include "config.h"
#include "pins.h"
#include "Ultrasonic.h"

// Minimale dlog()-Implementierung: schreibt direkt auf Serial, ohne
// CommChannel/PIDController/Settings zu benoetigen (die braucht nur die
// normale Firmware in src/myLogger.cpp).
void dlog(const String &msg)
{
    Serial.println(msg);
}

Ultrasonic ultrasonic;

void setup()
{
    Serial.begin(115200);
    delay(2000);
    LOG(">> Modus: ULTRASCHALL TEST");
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
            LOG_FMT("[ULTRA] Hoehe: %.1f cm", ultrasonic.getAltitudeCm());
        }
        else
        {
            LOG("[ULTRA] Kein Signal!");
        }
    }
}
