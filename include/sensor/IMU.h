#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <ICM20948_WE.h>

class IMU
{
public:
    bool begin(bool initWire = false);
    bool update();
    void calibrate();

    float getRoll()  const { return _roll; }
    float getPitch() const { return _pitch; }
    float getAccZ()  const { return _accelZ; }
    bool  isReady()  const { return _ready; }

private:
    ICM20948_WE _imu{0x68};

    float _roll   = 0.0f;
    float _pitch  = 0.0f;
    float _accelZ = 0.0f;
    bool  _ready  = false;

    uint32_t _lastUpdateUs = 0;
    bool     _firstUpdate  = true;
};
