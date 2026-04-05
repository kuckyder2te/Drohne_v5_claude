#include "sensor/IMU.h"
#include "pins.h"

bool IMU::begin()
{
    // Wire bereits initialisiert — direkt setup aufrufen
    if (!_mpu.setup(0x68))
    {
        Serial.println("[IMU] ERROR: MPU9250 nicht gefunden!");
        return false;
    }
    Serial.println("[IMU] MPU9250 gefunden");
    Serial.println("[IMU] Kalibrierung läuft — bitte ruhig halten...");
    delay(2000);
    _mpu.calibrateAccelGyro();
    Serial.println("[IMU] Accel/Gyro kalibriert");
    _ready = true;
    return true;
}

void IMU::calibrate()
{
    Serial.println("[IMU] Rekalibrierung...");
    _mpu.calibrateAccelGyro();
    Serial.println("[IMU] Fertig");
}

bool IMU::update() {
    if (!_ready) return false;
    _mpu.update();  // immer aufrufen
    _roll  = _mpu.getRoll();
    _pitch = _mpu.getPitch();
    _yaw   = _mpu.getYaw();
    return true;
}