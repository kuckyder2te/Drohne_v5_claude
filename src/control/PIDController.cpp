#include "control/PIDController.h"
#include "myLogger.h"
#include "config.h"

PIDController::PIDController(float kp, float ki, float kd, bool useOffset)
    : _kp(kp), _ki(ki), _kd(kd), _useOffset(useOffset) {}

float PIDController::_clampCoeff(float val, const char *name)
{
    if (val < PID_COEFF_MIN)
    {
        LOG_FMT("[PID] WARNUNG: %s", name);             // Serial.print(name);
        LOG_FMT(" zu klein → auf %.4f", PID_COEFF_MIN); // Serial.println(PID_COEFF_MIN, 4);
        return PID_COEFF_MIN;
    }
    if (val > PID_COEFF_MAX)
    {
        LOG_FMT("[PID] WARNUNG: %s", name);
        LOG_FMT(" zu groß → auf %.4f", PID_COEFF_MAX);
        return PID_COEFF_MAX;
    }
    return val;
}

void PIDController::begin()
{
    _kp = _clampCoeff(_kp, "Kp");
    _ki = _clampCoeff(_ki, "Ki");
    _kd = _clampCoeff(_kd, "Kd");
    reset();
    LOG("[PID] Regler initialisiert (eigene Implementierung)");
    LOG_FMT("[PID] Kp=%.4f", _kp);
    LOG_FMT("[PID] Ki=%.4f", _ki);
    LOG_FMT("[PID] Kd=%.4f", _kd);
}

float PIDController::compute(float setpoint, float measured)
{
    float now = millis() / 1000.0f;
    float dt = now - _lastTime;

    if (dt <= 0.0f || dt > 1.0f)
    {
        _lastTime = now;
        return _useOffset ? ESC_MIN_US : 0.0f; // ← 0 für Roll/Pitch!
    }

    float error = setpoint - measured;
    if (_integralEnabled) {
        _integral += error * dt;
        _integral = constrain(_integral, _integralMin, _integralMax);
    }
    float derivative = (error - _lastError) / dt;

    // Basis-Offset + PID Output
    float output = (_useOffset ? THROTTLE_OFFSET_US : 0.0f) + (_kp * error) + (_ki * _integral) + (_kd * derivative);

    if (_useOffset)
    {
        output = constrain(output, ESC_MIN_US, ESC_MAX_US);
    }
    else
    {
        output = constrain(output, -500.0f, 500.0f); // ← Korrekturbereich
    }

    _lastError = error;
    _lastTime = now;

    _lastThrottle = output;

    return output;
}

void PIDController::reset()
{
    _integral     = 0.0f;
    _lastError    = 0.0f;
    _lastTime     = millis() / 1000.0f;
    _lastThrottle = _useOffset ? (float)ESC_MIN_US : 0.0f;
    LOG("[PID] Reset");
}

void PIDController::setKp(float kp)
{
    _kp = _clampCoeff(kp, "Kp");
    LOG_FMT("[PID] Kp=%.4f", _kp);
}

void PIDController::setKi(float ki)
{
    _ki = _clampCoeff(ki, "Ki");
    LOG_FMT("[PID] Ki=%.4f", _ki);
}

void PIDController::setKd(float kd)
{
    _kd = _clampCoeff(kd, "Kd");
    LOG_FMT("[PID] Kd=%.4f", _kd);
}

void PIDController::enableIntegral(bool enable)
{
    if (_integralEnabled && !enable) {
        _integral = 0.0f; // Integral löschen beim Landen
        LOG("[PID] Integral deaktiviert (Landung)");
    } else if (!_integralEnabled && enable) {
        LOG("[PID] Integral aktiv (abgehoben)");
    }
    _integralEnabled = enable;
}