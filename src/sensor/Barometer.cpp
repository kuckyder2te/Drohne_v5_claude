#include "sensor/Barometer.h"
#include "myLogger.h"
#include "pins.h"

// MS5611 Kommandos
#define CMD_RESET 0x1E
#define CMD_CONV_D1 0x44 // Druck, OSR=1024
#define CMD_CONV_D2 0x54 // Temperatur, OSR=1024
#define CMD_READ_ADC 0x00
#define CMD_READ_PROM 0xA0

bool Barometer::begin()
{
    // 1. Variablen initialisieren
    _C0 = _C1 = _C2 = _C3 = _C4 = _C5 = _C6 = 0;

    // Kurze Pause für I2C Stabilisierung
    Wire.beginTransmission(MS5611_ADDR);
    Wire.endTransmission();
    delay(10); // ← statt Debug Log

    // Debug vor Reset
    // Wire.beginTransmission(MS5611_ADDR);
    // uint8_t err = Wire.endTransmission();
    // LOG_FMT("[BARO] I2C Test vor Reset: %d", err);

    // 2. Reset senden
    if (!_reset())
    {
        LOG("[BARO] ERROR: Reset fehlgeschlagen!");
        return false;
    }

    // 3. PROM lesen
    if (!_readPROM())
    {
        LOG("[BARO] ERROR: PROM lesen fehlgeschlagen!");
        return false;
    }

#ifdef TEST_BAROMETER
    LOG_FMT("[BARO] C1=%u C2=%u C3=%u C4=%u C5=%u C6=%u",
            _C1, _C2, _C3, _C4, _C5, _C6);

#endif

    LOG("[BARO] MS5611(MS5607) gefunden");

    // 5. Aufwärmzeit
    LOG("[BARO] Aufwaermzeit 30s...");
    for (int i = 30; i > 0; i--)
    {
        update();
        if (i % 10 == 0)
        {
            LOG_FMT("[BARO] Noch %d s...", i);
        }
        delay(1000);
    }

    // Warten bis Temperatur stabil
    LOG("[BARO] Warte auf thermische Stabilisierung...");
    float lastTemp = 0;
    int stableCount = 0;
    do
    {
        lastTemp = _temperature;
        update();
        delay(1000);
        if (abs(_temperature - lastTemp) < 0.05f)
        {
            stableCount++;
        }
        else
        {
            stableCount = 0;
        }
        LOG_FMT("[BARO] Temp: %.1f C | Stabil: %d/10", _temperature, stableCount);
    } while (stableCount < 10); // 5 stabile Messungen

    LOG("[BARO] Temperatur stabil!");

    calibrate();
    return true;
}

bool Barometer::_reset()
{
    Wire.beginTransmission(MS5611_ADDR);
    Wire.write(CMD_RESET);
    if (Wire.endTransmission() != 0)
        return false;
    delay(10);
    return true;
}

bool Barometer::_readPROM()
{
    uint16_t *C[] = {&_C0, &_C1, &_C2, &_C3, &_C4, &_C5, &_C6};

    for (uint8_t i = 0; i < 6; i++)
    {
        Wire.beginTransmission(MS5611_ADDR);
        Wire.write(0xA2 + i * 2); // 0xA2, 0xA4, 0xA6, 0xA8, 0xAA, 0xAC
        Wire.endTransmission();
        delay(20);

        Wire.requestFrom((uint8_t)MS5611_ADDR, (uint8_t)2);
        *C[i + 1] = ((uint16_t)Wire.read() << 8) | Wire.read();

        #ifdef TEST_BAROMETER
        LOG_FMT("[BARO] C%d = %u", i + 1, *C[i + 1]);
        #endif
    }
    return true;
}

uint32_t Barometer::_readRaw(uint8_t cmd)
{
    Wire.beginTransmission(MS5611_ADDR);
    Wire.write(cmd);
    Wire.endTransmission();
    delay(5); // ← von 20ms auf 5ms für OSR=1024

    Wire.beginTransmission(MS5611_ADDR);
    Wire.write(CMD_READ_ADC);
    Wire.endTransmission();
    delay(5); // ← kleine Pause

    uint8_t n = Wire.requestFrom((uint8_t)MS5611_ADDR, (uint8_t)3);
    if (n != 3)
        return 0;

    uint32_t val = ((uint32_t)Wire.read() << 16) |
                   ((uint32_t)Wire.read() << 8) |
                   Wire.read();
    return val;
}

void Barometer::update()
{
    uint32_t D1 = _readRaw(CMD_CONV_D1);
    uint32_t D2 = _readRaw(CMD_CONV_D2);

    // Plausibilitätsprüfung
    if (D1 == 0 || D2 == 0)
        return;
    if (D1 < 1000000 || D1 > 16000000)
        return;
    if (D2 < 1000000 || D2 > 16000000)
        return;

    // MS5607 Formel
    int32_t dT = (int32_t)D2 - (int32_t)((uint32_t)_C5 << 8);
    int32_t TEMP = 2000 + (int32_t)(((int64_t)dT * _C6) >> 23);

    int64_t OFF = ((int64_t)_C2 << 17) + (((int64_t)_C4 * dT) >> 6);
    int64_t SENS = ((int64_t)_C1 << 16) + (((int64_t)_C3 * dT) >> 7);
    int32_t P = (int32_t)(((((int64_t)D1 * SENS) >> 21) - OFF) >> 15);

    _temperature = TEMP / 100.0f;
    _pressure = P / 100.0f;

    // Temperaturkompensation
    float tempDiff = _temperature - _calTemp;
    float compensatedPressure = _pressure + (tempDiff * BARO_TEMP_COEFF);

    float ratio = compensatedPressure / _refPressure;
    float altitudeM = 44330.0f * (1.0f - pow(ratio, 0.1902949f));
    _altitudeCm = _applyFilter(altitudeM * 100.0f);
    delay(50);
}

void Barometer::calibrate()
{
    LOG("[BARO] Kalibrierung laeuft...");
    float sum = 0.0f;
    const int samples = 20;
    for (int i = 0; i < samples; i++)
    {
        update();
        sum += _pressure;
        delay(100);
    }
    _refPressure = sum / samples;
    _calTemp = _temperature; // ← hier! nach der Kalibrierung

    // Filter komplett zurücksetzen mit 0.0f
    for (uint8_t i = 0; i < BARO_FILTER_SIZE; i++)
    {
        _filterBuf[i] = 0.0f;
    }
    _filterIdx = 0;      // ← hinzufügen!
    _filterFull = false; // ← false statt true!

    LOG_FMT("[BARO] Referenzdruck: %.2f hPa", _refPressure);
}

float Barometer::_applyFilter(float newValue)
{
    _filterBuf[_filterIdx] = newValue;
    _filterIdx = (_filterIdx + 1) % BARO_FILTER_SIZE;
    if (_filterIdx == 0)
        _filterFull = true;

    uint8_t count = _filterFull ? BARO_FILTER_SIZE : _filterIdx;
    float sum = 0.0f;
    for (uint8_t i = 0; i < count; i++)
        sum += _filterBuf[i];
    return sum / count;
}