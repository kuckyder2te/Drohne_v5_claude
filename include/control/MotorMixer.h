#pragma once

#include <Arduino.h>
#include "pins.h"

class MotorMixer
{
public:
    void begin();
    void setThrottle(uint16_t throttle_us); // 1000–2000 µs
    void mix(uint16_t throttle, float roll, float pitch, float yaw);
    void setSingle(uint8_t motor, uint16_t throttle); // ← NEU: 1=FL 2=FR 3=BR 4=BL
    void stop();

private:
    uint16_t _fl = 1000, _fr = 1000, _bl = 1000, _br = 1000;
    uint16_t _throttle_us = 1000;

    void _writePWM(uint8_t pin, uint16_t us);
};
