#include "control/PIDController.h"
#include "myLogger.h"
#include "config.h"

PIDController::PIDController(float kp, float ki, float kd, bool useOffset)
    : _kp(kp), _ki(ki), _kd(kd), _useOffset(useOffset) {}

float PIDController::_clampCoeff(float val, const char *name)
{
    if (val < PID_COEFF_MIN)
    {
        if (!_quiet) {
            LOGGER_NOTICE_FMT("[PID] WARNUNG: %s", name);             // Serial.print(name);
            LOGGER_NOTICE_FMT(" zu klein → auf %.4f", PID_COEFF_MIN); // Serial.println(PID_COEFF_MIN, 4);
        }
        return PID_COEFF_MIN;
    }
    if (val > PID_COEFF_MAX)
    {
        if (!_quiet) {
            LOGGER_NOTICE_FMT("[PID] WARNUNG: %s", name);
            LOGGER_NOTICE_FMT(" zu groß → auf %.4f", PID_COEFF_MAX);
        }
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
    if (_quiet) return;
    LOGGER_NOTICE("[PID] Regler initialisiert (eigene Implementierung)");
    LOGGER_NOTICE_FMT("[PID] Kp=%.4f", _kp);
    LOGGER_NOTICE_FMT("[PID] Ki=%.4f", _ki);
    LOGGER_NOTICE_FMT("[PID] Kd=%.4f", _kd);
}

// Zeitbasis ist micros(), nicht mehr millis(): bei ATTITUDE_RATE_HZ betraegt
// ein Zyklus 2,5 ms, die Millisekunden-Aufloesung haette dt zwischen 2 und 3
// springen lassen - ein Fehler von bis zu 40 % direkt im D-Anteil.
float PIDController::compute(float setpoint, float measured)
{
    uint32_t now = micros();
    float dt = _firstRun ? 0.0f : (uint32_t)(now - _lastUs) * 1e-6f;
    _lastUs   = now;
    _firstRun = false;
    return _core(setpoint, measured, dt, false, 0.0f);
}

float PIDController::compute(float setpoint, float measured, float dt)
{
    _lastUs   = micros();
    _firstRun = false;
    return _core(setpoint, measured, dt, false, 0.0f);
}

float PIDController::computeWithRate(float setpoint, float measured, float rate, float dt)
{
    _lastUs   = micros();
    _firstRun = false;
    return _core(setpoint, measured, dt, true, rate);
}

float PIDController::_core(float setpoint, float measured, float dt, bool useRate, float rate)
{
    float error = setpoint - measured;

    // Untergrenze 100 us schuetzt den Differenzenquotienten vor einer
    // Division durch nahezu null; Obergrenze faengt Aussetzer ab.
    if (dt < 1e-4f || dt > 0.5f)
    {
        // Fehler trotzdem uebernehmen: sonst bildet der naechste regulaere
        // Zyklus die Differenz gegen einen veralteten Wert und erzeugt einen
        // kuenstlichen D-Ausschlag.
        _lastError = error;
        _lastP = _lastI = _lastD = 0.0f;
        _lastThrottle = _useOffset ? (float)ESC_MIN_US : 0.0f;
        return _lastThrottle;
    }

    if (_integralEnabled) {
        _integral += error * dt;
        _integral = constrain(_integral, _integralMin, _integralMax);
    }

    // Bei konstantem Sollwert ist d(error)/dt = -d(measured)/dt = -rate.
    float derivative = useRate ? -rate : (error - _lastError) / dt;

    _lastP = _kp * error;
    _lastI = _ki * _integral;
    _lastD = _kd * derivative;

    float output = (_useOffset ? THROTTLE_OFFSET_US : 0.0f) + _lastP + _lastI + _lastD;

    if (_useOffset)
    {
        output = constrain(output, (float)ESC_MIN_US, (float)ESC_MAX_US);
    }
    else
    {
        output = constrain(output, -500.0f, 500.0f); // ← Korrekturbereich
    }

    _lastError = error;
    _lastThrottle = output;

    return output;
}

void PIDController::reset()
{
    _integral     = 0.0f;
    _lastError    = 0.0f;
    _lastUs       = micros();
    _firstRun     = true;
    _lastP = _lastI = _lastD = 0.0f;
    _lastThrottle = _useOffset ? (float)ESC_MIN_US : 0.0f;
    if (!_quiet) LOGGER_NOTICE("[PID] Reset");
}

void PIDController::resyncTime()
{
    _lastUs   = micros();
    _firstRun = true;
}

void PIDController::setKp(float kp)
{
    _kp = _clampCoeff(kp, "Kp");
    if (!_quiet) LOGGER_NOTICE_FMT("[PID] Kp=%.4f", _kp);
}

void PIDController::setKi(float ki)
{
    _ki = _clampCoeff(ki, "Ki");
    if (!_quiet) LOGGER_NOTICE_FMT("[PID] Ki=%.4f", _ki);
}

void PIDController::setKd(float kd)
{
    _kd = _clampCoeff(kd, "Kd");
    if (!_quiet) LOGGER_NOTICE_FMT("[PID] Kd=%.4f", _kd);
}

void PIDController::enableIntegral(bool enable)
{
    if (_integralEnabled && !enable) {
        _integral = 0.0f; // Integral löschen beim Landen
        if (!_quiet) LOGGER_NOTICE("[PID] Integral deaktiviert (Landung)");
    } else if (!_integralEnabled && enable) {
        if (!_quiet) LOGGER_NOTICE("[PID] Integral aktiv (abgehoben)");
    }
    _integralEnabled = enable;
}
