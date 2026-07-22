#pragma once

#include <Arduino.h>
#include "control/PIDController.h"
#include "MotorMixer.h"

class Barometer;
class Ultrasonic;
class IMU;
class Battery;
class Settings;

// Buendelt Flugzustand (armed/targetHeightCm/...), Arm-/Disarm-Sequenz,
// Sicherheits-Check (IMU/Hoehensprung) und den kaskadierten PID-Regelkreis
// (Hoehe + Lage) inkl. Motor-Mixing - vorher direkt in main.cpp/InputHandler.
class FlightController {
public:
    void begin(Settings &settings);

    void requestArm(bool imuReady, Barometer &baro);
    void disarm();
    void recalibrate(Barometer &baro);
    void adjustTargetHeight(float deltaCm);
    void updateArmPendingTimeout();
    void toggleStatusLog();

    void checkSafety(bool imuReady, float baroAltitudeCm);
    void updateControlLoop(const Ultrasonic &ultrasonic, const Barometer &baro, const IMU &imu);
    void logStatus(const Battery &battery, const Barometer &baro, const Ultrasonic &ultrasonic);

    bool  isArmed()           const { return _armed; }
    float getTargetHeightCm() const { return _targetHeightCm; }
    void  setTargetHeightCm(float cm);

    PIDController &getPidHeight() { return _pidHeight; }
    PIDController &getPidRoll()   { return _pidRoll; }
    PIDController &getPidPitch()  { return _pidPitch; }

private:
    MotorMixer    _motors;
    PIDController _pidHeight{PID_KP_HEIGHT, PID_KI_HEIGHT, PID_KD_HEIGHT, true};  // mit Offset
    PIDController _pidRoll{PID_KP_ROLL, PID_KI_ROLL, PID_KD_ROLL, false};        // ohne Offset
    PIDController _pidPitch{PID_KP_PITCH, PID_KI_PITCH, PID_KD_PITCH, false};    // ohne Offset

    bool     _armed = false;
    bool     _statusLogEnabled = false;
    bool     _armPending = false;
    uint32_t _armPendingMs = 0;
    uint32_t _lastPidMs = 0;
    uint32_t _lastPrintMs = 0;
    float    _targetHeightCm = 0.0f;
    float    _lastSafetyHeightCm = 0.0f;
};
