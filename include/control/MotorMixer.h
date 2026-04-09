#pragma once

#include <Arduino.h>
#include "pins.h"

class MotorMixer
{
public:
    void begin();
    void setThrottle(uint16_t throttle_us); // 1000–2000 µs
    void mix(uint16_t throttle, float roll, float pitch, float yaw);
    void stop();

private:
    uint16_t _fl = 1000, _fr = 1000, _bl = 1000, _br = 1000;
    uint16_t getThrottle() const { return _throttle_us; }

private:
    void _writePWM(uint8_t pin, uint16_t us);
    uint16_t _throttle_us = 1000;
};