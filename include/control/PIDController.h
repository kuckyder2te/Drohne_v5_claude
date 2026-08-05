#pragma once

#include <Arduino.h>
#include "config.h"

#define PID_COEFF_MIN   0.0f
#define PID_COEFF_MAX   255.0f

class PIDController {
public:
    PIDController(float kp, float ki, float kd, bool useOffset = true);

    void  begin();

    // dt wird aus micros() gebildet.
    float compute(float setpoint, float measured);

    // dt vorgegeben - fuer den Lageregelkreis auf Kern 1, der seine
    // Zykluszeit ohnehin kennt.
    float compute(float setpoint, float measured, float dt);

    // D-Anteil aus einer gemessenen Rate statt aus dem Differenzenquotienten.
    // Bei ATTITUDE_RATE_HZ ist (error - lastError)/dt auf einem verrauschten
    // Winkel unbrauchbar; die Gyro-Rate IST die Ableitung des Winkels und
    // liegt sauber vor. Bei konstantem Sollwert gilt d(error)/dt = -rate,
    // daher geht die Rate negativ ein. Nebeneffekt: kein Derivative-Kick bei
    // Sollwertspruengen - den erzeugt das Relais beim Autotuning garantiert.
    float computeWithRate(float setpoint, float measured, float rate, float dt);

    float getLastThrottle() const { return _lastThrottle; }
    void  reset();
    void  enableIntegral(bool enable);

    void  setKp(float kp);
    void  setKi(float ki);
    void  setKd(float kd);
    float getKp() const { return _kp; }
    float getKi() const { return _ki; }
    float getKd() const { return _kd; }

    // Letzte Einzelbeitraege - der Recorder zeichnet sie auf, sonst laesst
    // sich hinterher nicht sagen, welcher Anteil eine Schwingung getrieben hat.
    float getP() const { return _lastP; }
    float getI() const { return _lastI; }
    float getD() const { return _lastD; }
    float getError()    const { return _lastError; }
    float getIntegral() const { return _integral; }

    // Ein stiller Wechsel der Zeitbasis ohne Sprung im D-Anteil: setzt
    // _lastUs neu, ohne Integral und Fehler zu verwerfen.
    void  resyncTime();

    // Schaltet saemtliche LOGGER_*-Ausgaben dieser Instanz ab. PFLICHT fuer
    // jede Instanz, die auf Kern 1 laeuft: die *_FMT-Makros schreiben in den
    // EINEN globalen logBuf aus src/myLogger.cpp - eine Ausgabe von Kern 1
    // wuerde eine gleichzeitig laufende von Kern 0 mitten im sprintf
    // zerstoeren. Setzen die Reglerinstanzen in AttitudeLoop.cpp.
    void  setQuiet(bool q) { _quiet = q; }

private:
    float _kp, _ki, _kd;
    float _lastThrottle = ESC_MIN_US;

    // PID Zustand
    float    _integral  = 0.0f;
    float    _lastError = 0.0f;
    uint32_t _lastUs    = 0;
    bool     _firstRun  = true;

    float _lastP = 0.0f, _lastI = 0.0f, _lastD = 0.0f;

    // Anti-Windup: Integral begrenzen + Liftoff-Guard
    float _integralMin     = -500.0f;
    float _integralMax     =  500.0f;
    bool  _useOffset       = true;
    bool  _integralEnabled = false;
    bool  _quiet           = false;

    float _clampCoeff(float val, const char* name);
    float _core(float setpoint, float measured, float dt, bool useRate, float rate);
};
