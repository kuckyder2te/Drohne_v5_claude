#include "IMU.h"
#include "myLogger.h"
#include "pins.h"

bool IMU::begin(bool initWire) {
    if (initWire) {
        // Bus-Recovery: 9 SCL-Pulse befreien haengenden SDA
        pinMode(PIN_SDA, OUTPUT);
        pinMode(PIN_SCL, OUTPUT);
        digitalWrite(PIN_SDA, HIGH);
        for (int i = 0; i < 9; i++) {
            digitalWrite(PIN_SCL, HIGH); delayMicroseconds(5);
            digitalWrite(PIN_SCL, LOW);  delayMicroseconds(5);
        }
        digitalWrite(PIN_SDA, LOW);  delayMicroseconds(5);
        digitalWrite(PIN_SCL, HIGH); delayMicroseconds(5);
        digitalWrite(PIN_SDA, HIGH); delayMicroseconds(5);

        Wire.setSDA(PIN_SDA);
        Wire.setSCL(PIN_SCL);
        Wire.begin();
        Wire.setClock(400000);
        delay(100);
    }

    if (!_imu.init()) {
        LOG("[IMU] ERROR: ICM-20948 nicht gefunden!");
        return false;
    }

    LOG("[IMU] Kalibrierung - bitte ruhig halten...");
    _imu.autoOffsets();

    _imu.setGyrRange(ICM20948_GYRO_RANGE_250);
    _imu.setGyrDLPF(ICM20948_DLPF_6);
    _imu.setAccRange(ICM20948_ACC_RANGE_2G);
    _imu.setAccDLPF(ICM20948_DLPF_6);

    _ready = true;
    LOG("[IMU] ICM-20948 bereit");
    return true;
}

void IMU::calibrate() {
    LOG("[IMU] Rekalibrierung - bitte ruhig halten...");
    _imu.autoOffsets();
    LOG("[IMU] Kalibrierung abgeschlossen");
}

bool IMU::update() {
    if (!_ready) return false;

    _imu.readSensor();

    xyzFloat gVal;
    xyzFloat gyr;
    _imu.getGValues(&gVal);    // in g
    _imu.getGyrValues(&gyr);  // in deg/s

    _accelZ = gVal.z * 9.81f;

    uint32_t now = micros();
    float dt = _firstUpdate ? 0.01f : (now - _lastUpdateUs) * 1e-6f;
    _lastUpdateUs = now;
    _firstUpdate  = false;

    float accelRoll  = atan2f(gVal.y, gVal.z) * 180.0f / M_PI;
    float accelPitch = atan2f(-gVal.x, sqrtf(gVal.y*gVal.y + gVal.z*gVal.z)) * 180.0f / M_PI;

    const float alpha = 0.98f;
    _roll  = alpha * (_roll  + gyr.x * dt) + (1.0f - alpha) * accelRoll;
    _pitch = alpha * (_pitch + gyr.y * dt) + (1.0f - alpha) * accelPitch;

    return true;
}
