#include "sensor/IMU.h"
#include "myLogger.h"
#include "pins.h"

bool IMU::begin() {
    // I2C Bus manuell freigeben (Bus-Stuck Recovery)
    pinMode(PIN_SDA, OUTPUT);
    pinMode(PIN_SCL, OUTPUT);

    // 9 Clock-Pulse senden um hängendes Gerät zu befreien
    digitalWrite(PIN_SDA, HIGH);
    for (int i = 0; i < 9; i++) {
        digitalWrite(PIN_SCL, HIGH);
        delayMicroseconds(5);
        digitalWrite(PIN_SCL, LOW);
        delayMicroseconds(5);
    }
    // STOP Condition
    digitalWrite(PIN_SDA, LOW);
    delayMicroseconds(5);
    digitalWrite(PIN_SCL, HIGH);
    delayMicroseconds(5);
    digitalWrite(PIN_SDA, HIGH);
    delayMicroseconds(5);

    delay(100);

    // Wire normal initialisieren
    Wire.setSDA(PIN_SDA);
    Wire.setSCL(PIN_SCL);
    Wire.begin();
    Wire.setClock(400000);
    delay(200);

    int status = _mpu.begin();
    LOG_FMT("[IMU] begin() status: %d", status);

    if (status < 0) {
        LOG_FMT("[IMU] ERROR: begin() = %d", status);
        return false;
    }
    LOG("[IMU] MPU9250 gefunden");
    _mpu.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
    _ready = true;
    LOG("[IMU] Bereit");
    return true;
}

void IMU::calibrate()
{
    // Kalibrierung: Nullpunkt der Winkel setzen
    Serial.println("[IMU] Nullpunkt gesetzt");
}

bool IMU::update() {
    if (!_ready) return false;

    // Rohwerte direkt lesen — ohne auf readSensor() zu warten
    _mpu.readSensor();

    float ax = _mpu.getAccelX_mss();
    float ay = _mpu.getAccelY_mss();
    float az = _mpu.getAccelZ_mss();

    // Nur aktualisieren wenn sich Werte geändert haben
    if (ax != _accelX || ay != _accelY || az != _accelZ) {
        _accelX = ax;
        _accelY = ay;
        _accelZ = az;
        _gyroX  = _mpu.getGyroX_rads();
        _gyroY  = _mpu.getGyroY_rads();
        _gyroZ  = _mpu.getGyroZ_rads();
        _calcAngles();
        return true;
    }
    return false;
}

void IMU::_calcAngles()
{
    _roll = atan2f(_accelY, _accelZ) * 180.0f / M_PI;
    _pitch = atan2f(-_accelX,
                    sqrtf(_accelY * _accelY + _accelZ * _accelZ)) *
             180.0f / M_PI;
    _yaw = 0.0f;
}