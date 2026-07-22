#pragma once

#include <Arduino.h>
#include <Wire.h>

#define BARO_FILTER_SIZE 5 // ← war 20
#define MS5611_ADDR 0x77

class Barometer
{
public:
    bool begin();
    void update();
    void calibrate();

    float getAltitudeCm() const { return _altitudeCm; }
    float getPressure() const { return _pressure; }
    float getTemperature() const { return _temperature; }

private:
    // PROM Kalibrierungsdaten
    uint16_t _C0, _C1, _C2, _C3, _C4, _C5, _C6;

    float _calTemp = 0.0f; // ← Temperatur bei Kalibrierung
    float _altitudeCm = 0.0f;
    float _pressure = 0.0f;
    float _temperature = 0.0f;
    float _refPressure = 1013.25f;

    // Ringpuffer Mittelwertfilter
    float _filterBuf[BARO_FILTER_SIZE] = {0};
    uint8_t _filterIdx = 0;
    bool _filterFull = false;

    // Interne Hilfsfunktionen
    bool _reset();
    bool _readPROM();
    uint32_t _readRaw(uint8_t cmd);
    float _applyFilter(float newValue);
};