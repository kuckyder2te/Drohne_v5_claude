// Eigenstaendiges Tool (frueher TEST_I2C_SCAN in config.h/TestModes.cpp).
// Bauen/Flashen: pio run -e test_i2c_scan --target upload && pio device monitor
#include <Arduino.h>
#include <Wire.h>
#include "myLogger.h"
#include "config.h"
#include "pins.h"

// Minimale dlog()-Implementierung: schreibt direkt auf Serial, ohne
// CommChannel/PIDController/Settings zu benoetigen (die braucht nur die
// normale Firmware in src/myLogger.cpp).
void dlog(const String &msg)
{
    Serial.println(msg);
}

namespace {
    void i2cBusRecovery()
    {
        Wire.end();             // I2C Peripheral freigeben → Pins als GPIO nutzbar
        delay(10);
        pinMode(PIN_SDA, OUTPUT);
        pinMode(PIN_SCL, OUTPUT);
        digitalWrite(PIN_SDA, HIGH);
        for (int i = 0; i < 9; i++)
        {
            digitalWrite(PIN_SCL, HIGH); delayMicroseconds(10);
            digitalWrite(PIN_SCL, LOW);  delayMicroseconds(10);
        }
        // STOP-Condition erzeugen
        digitalWrite(PIN_SDA, LOW);  delayMicroseconds(10);
        digitalWrite(PIN_SCL, HIGH); delayMicroseconds(10);
        digitalWrite(PIN_SDA, HIGH); delayMicroseconds(10);
        delay(20);
        Wire.setSDA(PIN_SDA);
        Wire.setSCL(PIN_SCL);
        Wire.begin();
        delay(50);
    }

    void i2cScan()
    {
        i2cBusRecovery();

        // SDA-Pegel prüfen: LOW nach Recovery = Hardwareproblem
        Wire.end();
        delay(5);
        pinMode(PIN_SDA, INPUT_PULLUP);
        delay(5);
        bool sdaOk = digitalRead(PIN_SDA);
        Wire.setSDA(PIN_SDA);
        Wire.setSCL(PIN_SCL);
        Wire.begin();

        if (!sdaOk)
        {
            LOG("[I2C] FEHLER: SDA bleibt LOW nach Recovery!");
            LOG("[I2C] -> Kurzschluss, defektes Geraet oder fehlendes Pull-up?");
            LOG("[I2C] -> LiPo + USB trennen, 10s warten, neu starten.");
            return;
        }

        LOG("[I2C] Scanne Bus...");
        int found = 0;
        for (uint8_t addr = 1; addr < 127; addr++)
        {
            // Doppelprüfung: endTransmission + ein Byte lesen (echter ACK-Test)
            Wire.beginTransmission(addr);
            if (Wire.endTransmission() != 0) continue;

            Wire.requestFrom((int)addr, 1);
            if (Wire.available() < 1) continue;
            Wire.read();

            LOG_FMT("[I2C] Gefunden: 0x%02X", addr);
            found++;
        }
        if (found == 0)
            LOG("[I2C] Kein Geraet!");
        LOG_FMT("[I2C] Gesamt: %d Geraet(e)", found);
    }
}

void setup()
{
    Serial.begin(115200);
    delay(2000);
    LOG(">> Modus: I2C SCAN TEST (alle 5 s)");
}

void loop()
{
    static uint32_t lastI2C = 0;
    if (millis() - lastI2C >= 5000)
    {
        lastI2C = millis();
        i2cScan();
    }
}
