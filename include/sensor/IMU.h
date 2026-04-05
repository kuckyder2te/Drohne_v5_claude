#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <MPU9250.h>

class IMU {
public:
    bool begin();
    bool update();
    void calibrate();

    float getRoll()  const { return _roll; }
    float getPitch() const { return _pitch; }
    float getYaw()   const { return _yaw; }

    float getAccX()  const { return _mpu.getAccX(); }
    float getAccY()  const { return _mpu.getAccY(); }
    float getAccZ()  const { return _mpu.getAccZ(); }

    bool  isReady()  const { return _ready; }

private:
    MPU9250 _mpu;
    float   _roll  = 0.0f;
    float   _pitch = 0.0f;
    float   _yaw   = 0.0f;
    bool    _ready = false;
};