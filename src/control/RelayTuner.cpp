#include "control/RelayTuner.h"

#include <math.h>

using namespace shared;

void RelayTuner::begin(const TuneCfg &cfg) {
    _cfg      = cfg;
    _u        = 0.0f;
    _done     = false;
    _abort    = ABORT_NONE;
    _startUs  = 0;
    _lastSwitchUs = 0;
    _switches = 0;
    _nEdges   = 0;
    _nPeriods = 0;
    _peakMax  = -1e9f;
    _peakMin  =  1e9f;
}

float RelayTuner::step(uint32_t nowUs, float angleDeg) {
    if (_done) return 0.0f;

    if (_startUs == 0) {
        _startUs = nowUs;
        _lastSwitchUs = nowUs;
        // Mit einem Anstoss starten, sonst bliebe das Relais bei exakt 0
        // Grad im Totband stehen und es entstuende nie eine Schwingung.
        _u = _cfg.h;
    }

    // ── Abbruchbedingungen ─────────────────────────────────────────────
    if (fabsf(angleDeg) > _cfg.limitDeg) { abort(ABORT_ANGLE);   return 0.0f; }

    if ((uint32_t)(nowUs - _startUs) > _cfg.timeoutMs * 1000UL) {
        abort(ABORT_TIMEOUT); return 0.0f;
    }

    // Schaltet das Relais nicht mehr, schwingt nichts - typisch, wenn h zu
    // klein ist oder die Achse mechanisch blockiert. Ohne diese Pruefung
    // liefe der Versuch stumm bis zum Timeout.
    if ((uint32_t)(nowUs - _lastSwitchUs) > TUNE_NOSWITCH_MS * 1000UL) {
        abort(ABORT_NOSWITCH); return 0.0f;
    }

    // ── Zweipunktregler mit Hysterese ──────────────────────────────────
    // Sollwert ist 0 Grad, also error = -angle.
    float e = -angleDeg;
    float prev = _u;

    if      (e >  _cfg.eps) _u =  _cfg.h;
    else if (e < -_cfg.eps) _u = -_cfg.h;
    // sonst: _u bleibt stehen -> das ist die Hysterese

    // Extremwerte zwischen zwei Umschaltungen mitfuehren
    if (angleDeg > _peakMax) _peakMax = angleDeg;
    if (angleDeg < _peakMin) _peakMin = angleDeg;

    // ── Umschaltung erkannt ────────────────────────────────────────────
    if (_u != prev && prev != 0.0f) {
        _lastSwitchUs = nowUs;
        ++_switches;

        // Nur die steigenden Flanken zaehlen als Periodenmarke - der
        // Abstand zweier gleichgerichteter Flanken ist eine volle Periode.
        if (_u > 0.0f) {
            if (_nEdges > 0) {
                uint32_t tu = nowUs - _upEdgeUs[_nEdges - 1];
                float    a  = (_peakMax - _peakMin) * 0.5f;

                // Die ersten Perioden verwerfen: bis dahin schwingt sich
                // die Strecke erst ein, Amplitude und Dauer sind noch nicht
                // aussagekraeftig.
                uint8_t doneEdges = _nEdges;   // Zahl bereits gesehener Perioden
                if (doneEdges > _cfg.warmup && _nPeriods < MAX_PERIODS) {
                    _periods[_nPeriods] = tu;
                    _amps[_nPeriods]    = a;
                    ++_nPeriods;
                }
            }

            if (_nEdges < MAX_PERIODS) _upEdgeUs[_nEdges++] = nowUs;
            else {
                // Ringverhalten: aelteste Flanke verwerfen
                for (uint8_t k = 1; k < MAX_PERIODS; ++k) _upEdgeUs[k-1] = _upEdgeUs[k];
                _upEdgeUs[MAX_PERIODS-1] = nowUs;
            }

            _peakMax = -1e9f;
            _peakMin =  1e9f;
        }

        if (_nPeriods >= _cfg.cycles) {
            _abort = ABORT_DONE;
            _done  = true;
            return 0.0f;
        }
    }

    return _u;
}

TuneResult RelayTuner::result() const {
    TuneResult r;
    r.h         = _cfg.h;
    r.eps       = _cfg.eps;
    r.n         = _nPeriods;
    r.abortCode = _abort;

    if (_nPeriods == 0) { r.valid = false; return r; }

    double sumT = 0.0, sumA = 0.0;
    uint32_t tmin = 0xFFFFFFFFu, tmax = 0;
    for (uint8_t k = 0; k < _nPeriods; ++k) {
        sumT += _periods[k];
        sumA += _amps[k];
        if (_periods[k] < tmin) tmin = _periods[k];
        if (_periods[k] > tmax) tmax = _periods[k];
    }

    float tuUs = (float)(sumT / _nPeriods);
    r.tu     = tuUs * 1e-6f;
    r.amp    = (float)(sumA / _nPeriods);
    r.spread = tuUs > 0.0f ? (float)(tmax - tmin) / tuUs : 1.0f;

    // Die Formel setzt a > eps voraus. Bleibt die Schwingung im Totband,
    // ist sie nicht auswertbar - dann muss h groesser oder eps kleiner werden.
    r.valid = (r.amp > _cfg.eps) && (r.tu > 0.0f) && (_abort == ABORT_DONE);
    return r;
}
