#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <MS5611.h>

class Barometer {
public:
    bool begin();
    void update();

    float getAltitudeCm() const { return _altitudeCm; }
    float getPressure()   const { return _pressure; }
    float getTemperature() const { return _temperature; }

    void calibrate();  // Referenzdruck auf aktuellen Wert setzen

private:
    MS5611 _ms5611;
    float  _altitudeCm    = 0.0f;
    float  _pressure      = 0.0f;
    float  _temperature   = 0.0f;
    float  _refPressure   = 101325.0f;  // Standard-Atmosphäre als Startwert
};