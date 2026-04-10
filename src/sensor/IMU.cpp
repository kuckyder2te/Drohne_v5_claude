#include "sensor/IMU.h"
#include "myLogger.h"
#include "pins.h"

// ── Hilfsfunktionen ───────────────────────────────────────

void IMU::_writeReg(uint8_t reg, uint8_t val) {
    Wire.beginTransmission((uint8_t)0x68);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

uint8_t IMU::_readReg(uint8_t reg) {
    Wire.beginTransmission((uint8_t)0x68);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)0x68, (uint8_t)1);
    if (Wire.available()) {
        return Wire.read();
    }
    return 0xFF;  // Fehler
}

// ── Initialisierung ───────────────────────────────────────

    bool IMU::begin(bool initWire) {
    if (initWire) {
        // Nur im TEST_IMU Modus: Bus-Recovery + Wire initialisieren
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
        Wire.setClock(100000);
        delay(100);
    }
    // Normalbetrieb: Wire bereits vom Barometer initialisiert — direkt weiter

    // Software Reset
    _writeReg(0x6B, 0x80);
    delay(100);
    _writeReg(0x6B, 0x00);
    delay(100);

    // WHO_AM_I prüfen
    uint8_t whoami = _readReg(0x75);
    LOG_FMT("[IMU] WHO_AM_I: 0x%02X", whoami);
    if (whoami != 0x71 && whoami != 0x73) {
        LOG("[IMU] ERROR: Unbekannter Chip!");
        return false;
    }

    _writeReg(0x1A, 0x04);
    _writeReg(0x1B, 0x18);
    _writeReg(0x1C, 0x18);

    _ready = true;
    LOG("[IMU] MPU9250 bereit");
    return true;
}


// ── Kalibrierung ──────────────────────────────────────────

void IMU::calibrate() {
    LOG("[IMU] Rekalibrierung — bitte ruhig halten...");
    // Gyro Bias über 100 Messungen mitteln
    float bx = 0, by = 0, bz = 0;
    for (int i = 0; i < 100; i++) {
        Wire.beginTransmission((uint8_t)0x68);
        Wire.write((uint8_t)0x43);  // GYRO_XOUT_H
        Wire.endTransmission(false);
        Wire.requestFrom((uint8_t)0x68, (uint8_t)6);
        if (Wire.available() >= 6) {
            int16_t gx = (Wire.read() << 8) | Wire.read();
            int16_t gy = (Wire.read() << 8) | Wire.read();
            int16_t gz = (Wire.read() << 8) | Wire.read();
            bx += gx;
            by += gy;
            bz += gz;
        }
        delay(5);
    }
    _gyroBiasX = bx / 100.0f;
    _gyroBiasY = by / 100.0f;
    _gyroBiasZ = bz / 100.0f;
    LOG("[IMU] Kalibrierung abgeschlossen");
}

// ── Update ────────────────────────────────────────────────

bool IMU::update() {
    if (!_ready) return false;

    Wire.beginTransmission((uint8_t)0x68);
    Wire.write((uint8_t)0x3B);
    uint8_t err = Wire.endTransmission(false);

    // Bus-Fehler → Recovery
 // Bus-Fehler → Recovery
if (err != 0) {
    LOG_FMT("[IMU] Bus Fehler: %d — Recovery", err);

    // Wire komplett beenden
    Wire.end();
    delay(20);

    // Manuelle Clock-Pulse — befreit haengenden Bus
    pinMode(PIN_SDA, OUTPUT);
    pinMode(PIN_SCL, OUTPUT);
    digitalWrite(PIN_SDA, HIGH);
    for (int i = 0; i < 9; i++) {
        digitalWrite(PIN_SCL, HIGH); delayMicroseconds(10);
        digitalWrite(PIN_SCL, LOW);  delayMicroseconds(10);
    }
    // STOP Condition
    digitalWrite(PIN_SDA, LOW);  delayMicroseconds(10);
    digitalWrite(PIN_SCL, HIGH); delayMicroseconds(10);
    digitalWrite(PIN_SDA, HIGH); delayMicroseconds(10);

    delay(20);

    // Wire neu starten
    Wire.setSDA(PIN_SDA);
    Wire.setSCL(PIN_SCL);
    Wire.begin();
    Wire.setClock(100000);
    delay(50);

    // Nochmal versuchen
    Wire.beginTransmission((uint8_t)0x68);
    Wire.write((uint8_t)0x3B);
    err = Wire.endTransmission(false);
    if (err != 0) {
        LOG("[IMU] Recovery fehlgeschlagen!");
        return false;
    }
    LOG("[IMU] Recovery erfolgreich");
}

    Wire.requestFrom((uint8_t)0x68, (uint8_t)14);

    if (Wire.available() >= 14) {
        int16_t ax_raw = (Wire.read() << 8) | Wire.read();
        int16_t ay_raw = (Wire.read() << 8) | Wire.read();
        int16_t az_raw = (Wire.read() << 8) | Wire.read();
        Wire.read(); Wire.read();
        int16_t gx_raw = (Wire.read() << 8) | Wire.read();
        int16_t gy_raw = (Wire.read() << 8) | Wire.read();
        int16_t gz_raw = (Wire.read() << 8) | Wire.read();
        while (Wire.available()) Wire.read();

        _accelX = ax_raw / 2048.0f * 9.81f;
        _accelY = ay_raw / 2048.0f * 9.81f;
        _accelZ = az_raw / 2048.0f * 9.81f;
        _gyroX = (gx_raw - _gyroBiasX) / 16.384f * (M_PI / 180.0f);
        _gyroY = (gy_raw - _gyroBiasY) / 16.384f * (M_PI / 180.0f);
        _gyroZ = (gz_raw - _gyroBiasZ) / 16.384f * (M_PI / 180.0f);

        _calcAngles();
        return true;
    }
    while (Wire.available()) Wire.read();
    return false;
}

// ── Winkelberechnung ──────────────────────────────────────

void IMU::_calcAngles() {
    _roll  = atan2f(_accelY, _accelZ) * 180.0f / M_PI;
    _pitch = atan2f(-_accelX,
             sqrtf(_accelY * _accelY + _accelZ * _accelZ)) * 180.0f / M_PI;
    _yaw   = 0.0f;  // Yaw aus Gyro — Phase 3
}