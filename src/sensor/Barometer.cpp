#include "sensor/Barometer.h"
#include "myLogger.h"
#include "pins.h"

bool Barometer::begin() {
    Wire.setSDA(PIN_SDA);
    Wire.setSCL(PIN_SCL);
    Wire.begin();

    if (!_ms5611.begin()) {
        LOG("[BARO] ERROR: MS5611 nicht gefunden!");
        return false;
    }

    _ms5611.setOversampling(OSR_LOW);
    LOG("[BARO] MS5611 gefunden");

    LOG("[BARO] Aufwaermzeit 90s...");
    for (int i = 90; i > 0; i--) {
        _ms5611.read();
        if (i % 10 == 0) {
            LOG_FMT("[BARO] Noch %d s...", i);
        }
        delay(1000);
    }

    delay(100);
    calibrate();
    return true;
}

void Barometer::calibrate() {
    LOG("[BARO] Kalibrierung laeuft...");
    float sum = 0.0f;
    const int samples = 20;
    for (int i = 0; i < samples; i++) {
        _ms5611.read();
        sum += _ms5611.getPressure();
        delay(100);
    }
    _refPressure = sum / samples;

    for (uint8_t i = 0; i < BARO_FILTER_SIZE; i++) {
        _filterBuf[i] = 0.0f;
    }
    _filterFull = true;

    LOG_FMT("[BARO] Referenzdruck: %.2f hPa", _refPressure);
}

float Barometer::_applyFilter(float newValue) {
    _filterBuf[_filterIdx] = newValue;
    _filterIdx = (_filterIdx + 1) % BARO_FILTER_SIZE;
    if (_filterIdx == 0) _filterFull = true;

    uint8_t count = _filterFull ? BARO_FILTER_SIZE : _filterIdx;
    float sum = 0.0f;
    for (uint8_t i = 0; i < count; i++) sum += _filterBuf[i];
    return sum / count;
}

void Barometer::update() {
    int result = _ms5611.read();
    if (result != MS5611_READ_OK) {
        LOG("[BARO] Lesefehler!");
        return;
    }

    _pressure    = _ms5611.getPressure();
    _temperature = _ms5611.getTemperature();

    float ratio     = _pressure / _refPressure;
    float altitudeM = 44330.0f * (1.0f - pow(ratio, 0.1902949f));
    _altitudeCm     = _applyFilter(altitudeM * 100.0f);
}