#include "sensor/Barometer.h"
#include "pins.h"

bool Barometer::begin()
{
    Wire.setSDA(PIN_SDA);
    Wire.setSCL(PIN_SCL);
    Wire.begin();

    if (!_ms5611.begin())
    {
        Serial.println("[BARO] ERROR: MS5611 nicht gefunden!");
        return false;
    }

    _ms5611.setOversampling(OSR_HIGH);
    Serial.println("[BARO] MS5611 gefunden");

    // Sensor aufwärmen lassen vor Kalibrierung
    Serial.println("[BARO] Aufwärmzeit 90s läuft...");
    for (int i = 90; i > 0; i--)
    {
        _ms5611.read();
        if (i % 10 == 0)
        {
            Serial.print("[BARO] Noch ");
            Serial.print(i);
            Serial.println(" s...");
        }
        delay(1000);
    }

    delay(100);
    calibrate();
    return true;
}

void Barometer::calibrate()
{
    Serial.println("[BARO] Kalibrierung läuft...");
    float sum = 0.0f;
    const int samples = 20;
    for (int i = 0; i < samples; i++)
    {
        _ms5611.read();
        sum += _ms5611.getPressure();
        delay(100);
    }
    _refPressure = sum / samples;

    // Filter mit Nullwert vorbelegen → kein Drift beim Start
    for (uint8_t i = 0; i < BARO_FILTER_SIZE; i++)
    {
        _filterBuf[i] = 0.0f;
    }
    _filterFull = true; // Filter sofort als voll markieren

    Serial.print("[BARO] Referenzdruck: ");
    Serial.print(_refPressure);
    Serial.println(" hPa");
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

void Barometer::update() {
    int result = _ms5611.read();
    if (result != MS5611_READ_OK) {
        Serial.println("[BARO] Lesefehler!");
        return;
    }

    _pressure    = _ms5611.getPressure();
    _temperature = _ms5611.getTemperature();

    float ratio     = _pressure / _refPressure;
    float altitudeM = 44330.0f * (1.0f - pow(ratio, 0.1902949f));
    _altitudeCm     = _applyFilter(altitudeM * 100.0f);
}