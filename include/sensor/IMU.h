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

    float getAccX()  const { return _accelX; }
    float getAccY()  const { return _accelY; }
    float getAccZ()  const { return _accelZ; }

    float getGyroX() const { return _gyroX; }
    float getGyroY() const { return _gyroY; }
    float getGyroZ() const { return _gyroZ; }

    bool isReady()   const { return _ready; }

private:
    MPU9250 _mpu{Wire, 0x68};

    float _roll   = 0.0f;
    float _pitch  = 0.0f;
    float _yaw    = 0.0f;
    float _accelX = 0.0f;
    float _accelY = 0.0f;
    float _accelZ = 0.0f;
    float _gyroX  = 0.0f;
    float _gyroY  = 0.0f;
    float _gyroZ  = 0.0f;
    bool  _ready  = false;

    // Einfache Lageberechnung aus Accelerometer
    void _calcAngles();
};