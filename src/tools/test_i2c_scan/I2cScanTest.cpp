// Eigenstaendiges Tool (frueher TEST_I2C_SCAN in config.h/TestModes.cpp).
// Bauen/Flashen: pio run -e test_i2c_scan --target upload && pio device monitor
#include <Arduino.h>
#include <Wire.h>
#include "myLogger.h"
#include "config.h"
#include "pins.h"

// Formatierpuffer der *_FMT-Makros. src/myLogger.cpp wird in dieser
// Umgebung nicht mitkompiliert (build_src_filter), die Definition muss
// also hier stehen - auch lib/ nutzt die Makros. Groesse muss zu der
// Deklaration in include/myLogger.h passen.
char logBuf[160];

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
            LOGGER_NOTICE("[I2C] FEHLER: SDA bleibt LOW nach Recovery!");
            LOGGER_NOTICE("[I2C] -> Kurzschluss, defektes Geraet oder fehlendes Pull-up?");
            LOGGER_NOTICE("[I2C] -> LiPo + USB trennen, 10s warten, neu starten.");
            return;
        }

        LOGGER_NOTICE("[I2C] Scanne Bus...");
        int found = 0;
        for (uint8_t addr = 1; addr < 127; addr++)
        {
            // Nur Adress-ACK pruefen (Standard-Scan-Technik). Ein zusaetzlicher
            // requestFrom()-Lesetest wurde hier bewusst entfernt: der MS5611
            // liefert ohne vorheriges Kommando (siehe Barometer::_readRaw())
            // keine gueltigen Daten und NACKt/liefert 0 Bytes, obwohl er per
            // Adress-ACK laengst als vorhanden bestaetigt ist - ein zusaetzlicher
            // Lesetest an dieser Stelle meldet ihn faelschlich als "nicht gefunden".
            Wire.beginTransmission(addr);
            if (Wire.endTransmission() != 0) continue;

            LOGGER_NOTICE_FMT("[I2C] Gefunden: 0x%02X", addr);
            found++;
        }
        if (found == 0)
            LOGGER_NOTICE("[I2C] Kein Geraet!");
        LOGGER_NOTICE_FMT("[I2C] Gesamt: %d Geraet(e)", found);
    }
}

void setup()
{
    Serial.begin(115200);

    // Vorgabe der Bibliothek ist WARNING - ohne diese Zeile bliebe jede
    // LOGGER_NOTICE-Ausgabe unsichtbar. Ausgabe laeuft ueber
    // Logger::defaultLog nach Serial, eine eigene Ausgabefunktion
    // braucht das Tool nicht.
    Logger::setLogLevel(Logger::NOTICE);
    delay(2000);
    LOGGER_NOTICE(">> Modus: I2C SCAN TEST (alle 5 s)");
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
