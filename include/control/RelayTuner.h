#pragma once

#include <Arduino.h>
#include "control/SharedState.h"

// ── Relay-Feedback-Autotune nach Aastroem-Haegglund ────────────────────
//
// Statt eines PID-Reglers wirkt ein Zweipunktregler mit Hysterese auf die
// Achse. Der treibt die Regelstrecke von selbst in eine Dauerschwingung -
// und zwar genau bei der kritischen Frequenz, an der ein P-Regler an der
// Stabilitaetsgrenze staende. Aus dieser Schwingung folgen die beiden
// Groessen, die jede Einstellregel braucht:
//
//   Tu = Periodendauer der Schwingung
//   Ku = 4h / (pi * sqrt(a^2 - eps^2))     (h = Relaisamplitude, a = Halbamplitude)
//
// Der Vorteil gegenueber dem klassischen "Kp erhoehen bis es schwingt":
// die Amplitude bleibt durch h begrenzt und waechst nicht unkontrolliert.
//
// Einheitenprobe: h steht in us (Mixer-Korrektur), a in Grad, also hat Ku
// die Einheit us/Grad. Genau das ist die Einheit von Kp in diesem Code
// (rollCorr = Kp * error[Grad] geht direkt in mix() ein). Ku ist damit
// unmittelbar mit Kp vergleichbar.
class RelayTuner {
public:
    void begin(const shared::TuneCfg &cfg);

    // Ein Regelzyklus auf Kern 1. Liefert die Relaisstellung in us zurueck.
    float step(uint32_t nowUs, float angleDeg);

    bool     isDone()    const { return _done; }
    uint16_t abortCode() const { return _abort; }

    // Nach isDone(): das Messergebnis. Die Umrechnung auf Kp/Ki/Kd macht
    // Kern 0 - sie braucht kein Echtzeitverhalten.
    shared::TuneResult result() const;

    void abort(uint16_t code) { _abort = code; _done = true; }

private:
    static constexpr uint8_t MAX_PERIODS = 16;

    shared::TuneCfg _cfg;

    float    _u        = 0.0f;   // aktuelle Relaisstellung
    bool     _done     = false;
    uint16_t _abort    = shared::ABORT_NONE;

    uint32_t _startUs  = 0;
    uint32_t _lastSwitchUs = 0;
    uint8_t  _switches = 0;      // Zahl der Umschaltungen seit Start

    // Zeitpunkte der Umschaltungen mit gleichem Vorzeichen -> volle Perioden
    uint32_t _upEdgeUs[MAX_PERIODS];
    uint8_t  _nEdges   = 0;

    float    _peakMax  = -1e9f;
    float    _peakMin  =  1e9f;
    float    _amps[MAX_PERIODS];
    uint32_t _periods[MAX_PERIODS];
    uint8_t  _nPeriods = 0;
};
