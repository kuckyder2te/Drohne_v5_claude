// Eigenstaendiges Tool (frueher TEST_IMU in config.h/TestModes.cpp).
// Bauen/Flashen: pio run -e test_imu --target upload && pio device monitor
#include <Arduino.h>
#include "myLogger.h"
#include "config.h"
#include "pins.h"
#include "IMU.h"

// Minimale dlog()-Implementierung: schreibt direkt auf Serial, ohne
// CommChannel/PIDController/Settings zu benoetigen (die braucht nur die
// normale Firmware in src/myLogger.cpp).
void dlog(const String &msg)
{
    Serial.println(msg);
}

IMU imu;

void setup()
{
    Serial.begin(115200);
    delay(2000);
    LOG(">> Modus: IMU TEST");

    delay(1000);

    if (!imu.begin(true))
    {
        LOG("FEHLER: IMU! Programm gestoppt.");
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
        LOG_FMT("[IMU] Roll: %.1f  Pitch: %.1f  AccZ: %.2f  ready:%d",
                imu.getRoll(), imu.getPitch(), imu.getAccZ(), imu.isReady());
    }
}
