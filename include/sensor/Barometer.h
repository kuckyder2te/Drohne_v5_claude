#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <MS5611.h>

#define BARO_FILTER_SIZE 20

class Barometer {
public:
    bool begin();
    

    void update();

    float getAltitudeCm()  const { return _altitudeCm; }
    float getPressure()    const { return _pressure; }
    float getTemperature() const { return _temperature; }

    void calibrate();


private:
    MS5611  _ms5611;
    float   _altitudeCm  = 0.0f;
    float   _pressure    = 0.0f;
    float   _temperature = 0.0f;
    float   _refPressure = 1013.25f;  // hPa — war fälschlich Pa

    // Ringpuffer Mittelwertfilter
    float   _filterBuf[BARO_FILTER_SIZE] = {0};
    uint8_t _filterIdx  = 0;
    bool    _filterFull = false;

    bool _autoCorrect = true;

    float _applyFilter(float newValue);
};