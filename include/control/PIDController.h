#pragma once

#include <Arduino.h>
#include "config.h"

#define PID_HZ          20.0f
#define PID_COEFF_MIN   0.0f
#define PID_COEFF_MAX   255.0f

class PIDController {
public:
    PIDController(float kp = 1.0f, float ki = 0.05f, float kd = 0.1f);

    void  begin();
    float compute(float setpoint, float measured);
    float getLastThrottle() const { return _lastThrottle; }
    void  reset();

    void  setKp(float kp);
    void  setKi(float ki);
    void  setKd(float kd);
    float getKp() const { return _kp; }
    float getKi() const { return _ki; }
    float getKd() const { return _kd; }

private:
    float _kp, _ki, _kd;
    float _lastThrottle = ESC_MIN_US;

    // PID Zustand
    float _integral    = 0.0f;
    float _lastError   = 0.0f;
    float _lastTime    = 0.0f;

    // Anti-Windup: Integral begrenzen
    float _integralMin = -500.0f;
    float _integralMax =  500.0f;

    float _clampCoeff(float val, const char* name);
};