#pragma once

#include <Arduino.h>
#include "control/PIDController.h"
#include "control/SharedState.h"
#include "MotorMixer.h"

class Barometer;
class Ultrasonic;
class IMU;
class Battery;
class Settings;

// Buendelt Flugzustand (armed/targetHeightCm/...), Arm-/Disarm-Sequenz,
// Sicherheits-Check und den Hoehenregler. Der LAGEREGLER (Roll/Pitch) laeuft
// seit dem Dual-Core-Umbau auf Kern 1 (siehe control/AttitudeLoop.h) - dieser
// Klasse bleibt davon die Rolle, die Sollwerte und Koeffizienten dorthin zu
// veroeffentlichen.
class FlightController {
public:
    void begin(Settings &settings);

    void requestArm(bool imuReady, Barometer &baro);
    void disarm();
    void recalibrate(Barometer &baro);
    void adjustTargetHeight(float deltaCm);
    void updateArmPendingTimeout();
    void toggleStatusLog();

    void checkSafety(bool imuReady, float altitudeCm);
    void updateControlLoop(const Ultrasonic &ultrasonic, const Barometer &baro);
    void logStatus(const Battery &battery, const Barometer &baro, const Ultrasonic &ultrasonic);

    bool  isArmed()           const { return _armed; }
    float getTargetHeightCm() const { return _targetHeightCm; }
    void  setTargetHeightCm(float cm);

    PIDController &getPidHeight() { return _pidHeight; }
    PIDController &getPidRoll()   { return _pidRoll; }
    PIDController &getPidPitch()  { return _pidPitch; }

    // Not-Aus: schreibt die PWM-Register selbst, ohne auf Kern 1 zu warten.
    MotorMixer &getMotors() { return _motors; }

    // ── Bruecke zu Kern 1 ──────────────────────────────────────────────
    // Baut den Befehlsblock aus dem aktuellen Zustand und veroeffentlicht
    // ihn. Muss nach jeder Aenderung an Sollwerten, Koeffizienten oder
    // Betriebsart gerufen werden - die CLI tut das ueber publish().
    void publish();

    // Pruefstand/Autotune. Ein Aufruf setzt die Betriebsart und startet
    // ueber runId einen neuen Lauf auf Kern 1.
    void startBench(uint8_t axis, float throttleUs, float maxUs,
                    const shared::BenchCfg &cfg);
    void startTune(uint8_t axis, float throttleUs, float maxUs,
                   const shared::TuneCfg &cfg);
    void stopRun();

    uint8_t getMode() const { return _mode; }

private:
    MotorMixer    _motors;
    PIDController _pidHeight{PID_KP_HEIGHT, PID_KI_HEIGHT, PID_KD_HEIGHT, true};  // mit Offset

    // Diese beiden rechnen NICHT mehr - compute() wird nie gerufen. Sie
    // bleiben als Koeffizientenspeicher bestehen, weil die gesamte
    // pid-Logik der CLI (axisPid(), das Drei-Pass-Verfahren, printAxisJson)
    // ueber PIDController& arbeitet und so unveraendert weiterlaeuft. Die
    // Werte gehen ueber publish() an die Reglerinstanzen auf Kern 1.
    PIDController _pidRoll{PID_KP_ROLL, PID_KI_ROLL, PID_KD_ROLL, false};
    PIDController _pidPitch{PID_KP_PITCH, PID_KI_PITCH, PID_KD_PITCH, false};

    bool     _armed = false;
    bool     _statusLogEnabled = false;
    bool     _armPending = false;
    uint32_t _armPendingMs = 0;
    uint32_t _lastPidMs = 0;
    uint32_t _lastPrintMs = 0;
    float    _targetHeightCm = 0.0f;
    float    _lastSafetyHeightCm = 0.0f;

    uint8_t  _mode       = shared::MODE_IDLE;
    uint8_t  _axis       = shared::AXIS_ROLL;
    float    _throttleUs = ESC_MIN_US;
    float    _motorMaxUs = ESC_MAX_US;
    bool     _integralOn = false;
    uint32_t _runId      = 0;
    shared::BenchCfg _bench;
    shared::TuneCfg  _tune;
};
