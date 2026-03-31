#include "control/PIDController.h"
#include "config.h"

PIDController::PIDController(float kp, float ki, float kd)
    : _kp(kp), _ki(ki), _kd(kd) {}

float PIDController::_clampCoeff(float val, const char* name) {
    if (val < PID_COEFF_MIN) {
        Serial.print("[PID] WARNUNG: "); Serial.print(name);
        Serial.print(" zu klein → auf "); Serial.println(PID_COEFF_MIN, 4);
        return PID_COEFF_MIN;
    }
    if (val > PID_COEFF_MAX) {
        Serial.print("[PID] WARNUNG: "); Serial.print(name);
        Serial.print(" zu groß → auf "); Serial.println(PID_COEFF_MAX, 2);
        return PID_COEFF_MAX;
    }
    return val;
}

void PIDController::begin() {
    _kp = _clampCoeff(_kp, "Kp");
    _ki = _clampCoeff(_ki, "Ki");
    _kd = _clampCoeff(_kd, "Kd");
    reset();
    Serial.println("[PID] Regler initialisiert (eigene Implementierung)");
    Serial.print("[PID] Kp="); Serial.print(_kp, 4);
    Serial.print(" Ki=");      Serial.print(_ki, 4);
    Serial.print(" Kd=");      Serial.println(_kd, 4);
}

float PIDController::compute(float setpoint, float measured) {
    float now     = millis() / 1000.0f;  // Sekunden
    float dt      = now - _lastTime;

    // dt absichern — beim ersten Aufruf oder nach reset()
    if (dt <= 0.0f || dt > 1.0f) {
        _lastTime = now;
        return ESC_MIN_US;
    }

    float error = setpoint - measured;

    // Integral mit Anti-Windup
    _integral += error * dt;
    _integral  = constrain(_integral, _integralMin, _integralMax);

    // Derivative
    float derivative = (error - _lastError) / dt;

    // PID Output
    float output = (_kp * error) + (_ki * _integral) + (_kd * derivative);

    // Auf ESC-Bereich begrenzen
    output = constrain(output + ESC_MIN_US, ESC_MIN_US, ESC_MAX_US);

    _lastError = error;
    _lastTime  = now;

    return output;
}

void PIDController::reset() {
    _integral  = 0.0f;
    _lastError = 0.0f;
    _lastTime  = millis() / 1000.0f;
    Serial.println("[PID] Reset");
}

void PIDController::setKp(float kp) {
    _kp = _clampCoeff(kp, "Kp");
    Serial.print("[PID] Kp="); Serial.println(_kp, 4);
}

void PIDController::setKi(float ki) {
    _ki = _clampCoeff(ki, "Ki");
    Serial.print("[PID] Ki="); Serial.println(_ki, 4);
}

void PIDController::setKd(float kd) {
    _kd = _clampCoeff(kd, "Kd");
    Serial.print("[PID] Kd="); Serial.println(_kd, 4);
}