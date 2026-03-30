#include <Arduino.h>
#include "config.h"
#include "control/MotorMixer.h"
#include "sensor/Barometer.h"

MotorMixer motors;
Barometer baro;

#ifdef TEST_MOTORS
uint16_t currentThrottle = ESC_MIN_US;

void printHelp()
{
    Serial.println("─────────────────────────────");
    Serial.println(" MOTORTEST Befehle:");
    Serial.println("  + = Throttle +50 µs");
    Serial.println("  - = Throttle -50 µs");
    Serial.println("  s = STOP (1000 µs)");
    Serial.println("  h = Diese Hilfe");
    Serial.println("─────────────────────────────");
}
#endif

void i2cScan()
{
    Serial.println("[I2C] Scanne Bus...");
    int found = 0;
    for (uint8_t addr = 1; addr < 127; addr++)
    {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0)
        {
            Serial.print("[I2C] Gerät gefunden: 0x");
            Serial.println(addr, HEX);
            found++;
        }
    }
    if (found == 0)
        Serial.println("[I2C] Kein Gerät gefunden!");
}

void setup()
{
    Serial.begin(115200);
    delay(2000);
    Serial.println("=== DROHNE PICO BOOT ===");

#ifdef TEST_I2C_SCAN
    Wire.setSDA(PIN_SDA);
    Wire.setSCL(PIN_SCL);
    Wire.begin();
    i2cScan();
#endif

#ifdef TEST_MOTORS
    Serial.println(">> Modus: MOTORTEST");
    printHelp();
    motors.begin();
#endif

#ifdef TEST_BAROMETER
    Serial.println(">> Modus: BAROMETER TEST");
    if (!baro.begin())
    {
        Serial.println("FEHLER: Programm gestoppt.");
        while (true)
            delay(1000);
    }
    Serial.println("Bereit. Ausgabe alle 500ms:");
#endif
}

void loop()
{
#ifdef TEST_MOTORS
    if (Serial.available())
    {
        char cmd = Serial.read();
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
            printHelp();
            break;
        }
    }
#endif

#ifdef TEST_BAROMETER
    baro.update();
    Serial.print("[BARO] Höhe: ");
    Serial.print(baro.getAltitudeCm(), 1);
    Serial.print(" cm  |  Druck: ");
    Serial.print(baro.getPressure(), 2);
    Serial.print(" Pa  |  Temp: ");
    Serial.print(baro.getTemperature(), 1);
    Serial.println(" °C");
    delay(500);
#endif
}