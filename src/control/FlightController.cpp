#include "control/FlightController.h"

#include "myLogger.h"
#include "config.h"
#include "Barometer.h"
#include "Ultrasonic.h"
#include "IMU.h"
#include "Battery.h"
#include "storage/Settings.h"

namespace
{
    void applyCoeffs(PIDController &pid, const PidCoeffs &c)
    {
        pid.setKp(c.kp);
        pid.setKi(c.ki);
        pid.setKd(c.kd);
    }
}

void FlightController::begin(Settings &settings)
{
    _motors.begin();
    _pidHeight.begin();
    _pidRoll.begin();
    _pidPitch.begin();

    settings.begin();
    PidCoeffs height, roll, pitch;
    if (settings.load(height, roll, pitch))
    {
        applyCoeffs(_pidHeight, height);
        applyCoeffs(_pidRoll,   roll);
        applyCoeffs(_pidPitch,  pitch);
    }
}

void FlightController::requestArm(bool imuReady, Barometer &baro)
{
    if (_armed)
        return;

    if (!_armPending)
    {
        _armPending = true;
        _armPendingMs = millis();
        LOGGER_NOTICE("[CTRL] ARM? Nochmal 'a' druecken (3s)");
        return;
    }

    if (millis() - _armPendingMs > 3000)
    {
        _armPending = false;
        LOGGER_NOTICE("[CTRL] ARM abgebrochen (Timeout)");
        return;
    }

    _armPending = false;
    if (!imuReady)
    {
        LOGGER_NOTICE("[CTRL] ARM verweigert - IMU nicht bereit!");
        return;
    }

    LOGGER_NOTICE("[CTRL] Rekalibrierung vor ARM...");
    baro.calibrate();
    delay(500);
    _armed = true;
    _targetHeightCm = 20.0f;
    _lastPidMs = millis();
    _pidHeight.reset();
    _pidRoll.reset();
    _pidPitch.reset();
    LOGGER_NOTICE("[CTRL] ARM - Ziel: 20 cm");
}

void FlightController::disarm()
{
    _armed = false;
    _targetHeightCm = 0.0f;
    _motors.stop();
    _pidHeight.reset();
    _pidRoll.reset();
    _pidPitch.reset();
    LOGGER_NOTICE("[CTRL] DISARM - Motoren gestoppt");
}

void FlightController::recalibrate(Barometer &baro)
{
    if (_armed)
    {
        LOGGER_NOTICE("[CTRL] Rekalibrierung nur im DISARM Modus!");
        return;
    }
    baro.calibrate();
    _pidHeight.reset();
    LOGGER_NOTICE("[CTRL] Barometer rekalibriert");
}

void FlightController::adjustTargetHeight(float deltaCm)
{
    _targetHeightCm = constrain(_targetHeightCm + deltaCm, THROTTLE_MIN_CM, MAX_HEIGHT_CM);
    LOGGER_NOTICE_FMT("[CTRL] Zielhoehe: %.1f cm", _targetHeightCm);
}

void FlightController::updateArmPendingTimeout()
{
    if (_armPending && (millis() - _armPendingMs > 3000))
    {
        _armPending = false;
        LOGGER_NOTICE("[CTRL] ARM abgebrochen (Timeout)");
    }
}

void FlightController::toggleStatusLog()
{
    _statusLogEnabled = !_statusLogEnabled;
    LOGGER_NOTICE_FMT("[CTRL] Statusausgabe: %s", _statusLogEnabled ? "EIN" : "AUS");
}

void FlightController::setTargetHeightCm(float cm)
{
    _targetHeightCm = constrain(cm, THROTTLE_MIN_CM, MAX_HEIGHT_CM);
    LOGGER_NOTICE_FMT("[CTRL] Zielhoehe: %.1f cm", _targetHeightCm);
}

void FlightController::checkSafety(bool imuReady, float baroAltitudeCm)
{
    if (!_armed)
        return;

    if (!imuReady)
    {
        LOGGER_NOTICE("[SAFETY] IMU Fehler - DISARM!");
        disarm();
    }

    if (abs(baroAltitudeCm - _lastSafetyHeightCm) > 500.0f)
    {
        LOGGER_NOTICE("[SAFETY] Hoehensprung - DISARM!");
        disarm();
    }
    _lastSafetyHeightCm = baroAltitudeCm;
}

void FlightController::updateControlLoop(const Ultrasonic &ultrasonic, const Barometer &baro, const IMU &imu)
{
    if (!_armed || (millis() - _lastPidMs < PID_INTERVAL_MS))
        return;
    _lastPidMs = millis();

    bool airborne = ultrasonic.isValid() && (ultrasonic.getAltitudeCm() > LIFTOFF_HEIGHT_CM);
    _pidHeight.enableIntegral(airborne);
    _pidRoll.enableIntegral(airborne);
    _pidPitch.enableIntegral(airborne);

    float currentHeight = ultrasonic.isValid() ? ultrasonic.getAltitudeCm() : baro.getAltitudeCm();
    float throttle = _pidHeight.compute(_targetHeightCm, currentHeight);

    float rollCorr = _pidRoll.compute(TARGET_ROLL_DEG, imu.getRoll());
    float pitchCorr = _pidPitch.compute(TARGET_PITCH_DEG, imu.getPitch());

    _motors.mix((uint16_t)throttle, rollCorr, pitchCorr, 0.0f);
}

void FlightController::logStatus(const Battery &battery, const Barometer &baro, const Ultrasonic &ultrasonic)
{
    if (!_statusLogEnabled || (millis() - _lastPrintMs < 500))
        return;
    _lastPrintMs = millis();

    LOGGER_NOTICE_FMT("[CTRL] Ziel: %.1f cm | Ist: %.1f cm | Throttle: %.0f us | Armed: %s | Bat: %.2fV | Druck: %.2f hPa",
            _targetHeightCm,
            ultrasonic.isValid() ? ultrasonic.getAltitudeCm() : baro.getAltitudeCm(),
            _pidHeight.getLastThrottle(),
            _armed ? "JA" : "NEIN",
            battery.getVoltage(),
            baro.getPressure());
}
