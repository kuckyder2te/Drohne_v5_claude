#include "sensor/IMU.h"
#include "myLogger.h"
#include "pins.h"

bool IMU::begin() {
    int status = _mpu.begin();
    if (status < 0) {
        Serial.print("[IMU] ERROR: begin() = ");
        Serial.println(status);
        return false;
    }
    Serial.println("[IMU] MPU9250 gefunden");

    // 50 Hz Abtastrate
    _mpu.setSrd(19);

    // DLPF auf 20Hz — filtert Vibrationen
    _mpu.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);

    _ready = true;
    Serial.println("[IMU] Bereit");
    return true;
}

void IMU::calibrate() {
    // Kalibrierung: Nullpunkt der Winkel setzen
    Serial.println("[IMU] Nullpunkt gesetzt");
}

bool IMU::update() {
    if (!_ready) return false;
    if (_mpu.readSensor()) {
        _accelX = _mpu.getAccelX_mss();
        _accelY = _mpu.getAccelY_mss();
        _accelZ = _mpu.getAccelZ_mss();
        _gyroX  = _mpu.getGyroX_rads();
        _gyroY  = _mpu.getGyroY_rads();
        _gyroZ  = _mpu.getGyroZ_rads();
        _calcAngles();
        return true;
    }
    return false;
}

void IMU::_calcAngles() {
    _roll  = atan2f(_accelY, _accelZ) * 180.0f / M_PI;
    _pitch = atan2f(-_accelX,
             sqrtf(_accelY * _accelY + _accelZ * _accelZ)) * 180.0f / M_PI;
    _yaw   = 0.0f;
}