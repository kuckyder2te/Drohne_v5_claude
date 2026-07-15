#pragma once

#include <Arduino.h>

#define ULTRASONIC_MAX_CM    300.0f  // max 3m
#define ULTRASONIC_MIN_CM      2.0f  // min 2cm
#define ULTRASONIC_FILTER_SIZE   5   // Mittelwertfilter

class Ultrasonic {
public:
    void begin();
    void update();

    float getAltitudeCm() const { return _altitudeCm; }
    bool  isValid()       const { return _valid; }

private:
    float    _altitudeCm = 0.0f;
    bool     _valid      = false;

    // Ringpuffer Filter
    float    _filterBuf[ULTRASONIC_FILTER_SIZE] = {0};
    uint8_t  _filterIdx  = 0;
    bool     _filterFull = false;

    void  _trigger();
    float _measureCm(uint8_t echoPin);
    float _applyFilter(float newValue);
};
