#include "control/AttitudeLoop.h"

#include "control/SharedState.h"
#include "control/PIDController.h"
#include "control/RelayTuner.h"
#include "control/Recorder.h"
#include "config.h"
#include "IMU.h"
#include "MotorMixer.h"

using namespace shared;

namespace attitude {

namespace {
    IMU        *s_imu    = nullptr;
    MotorMixer *s_motors = nullptr;

    // Eigene Reglerinstanzen. Die Koeffizienten kommen ueber CoreCmd von
    // Kern 0; FlightController haelt dort weiterhin gleichlautende Objekte,
    // damit die gesamte pid-Logik der CLI unveraendert weiterlaeuft.
    PIDController s_pidRoll {PID_KP_ROLL,  PID_KI_ROLL,  PID_KD_ROLL,  false};
    PIDController s_pidPitch{PID_KP_PITCH, PID_KI_PITCH, PID_KD_PITCH, false};

    // Nur fuer den Diagonal-Pruefstand. Faehrt die Roll-Beiwerte durch
    // DIAG_AXIS_GAIN geteilt, damit die Wippe physikalisch dasselbe
    // Regelverhalten zeigt wie der Rollregler im Flug - auf der Diagonalen
    // wirken nur zwei Motoren, dafuer mit groesserem Hebel (Herleitung in
    // config.h). Ohne diese Hochrechnung waere die Wippe um Faktor sqrt(2)
    // zu weich eingestellt und der Prueflauf wuerde etwas anderes zeigen als
    // das, was spaeter fliegt.
    PIDController s_pidDiag {PID_KP_ROLL,  PID_KI_ROLL,  PID_KD_ROLL,  false};

    RelayTuner s_tuner;

    CoreCmd  s_cmd;              // letzte bekannte Fassung
    uint32_t s_lastRunId  = 0;
    uint8_t  s_lastMode   = MODE_IDLE;
    uint32_t s_runStartMs = 0;
    bool     s_running    = false;
    uint16_t s_abort      = ABORT_NONE;

    uint32_t s_nextUs     = 0;
    uint32_t s_lastUs     = 0;
    uint32_t s_overruns   = 0;
    uint32_t s_maxLoopUs  = 0;   // laufendes Sekundenfenster
    uint32_t s_maxHeld    = 0;   // Spitzenwert der abgelaufenen Sekunde
    uint32_t s_rateCount  = 0;
    uint32_t s_rateMs     = 0;
    uint32_t s_loopHz     = 0;
    uint8_t  s_angleTrips = 0;

    void applyCoeffs(PIDController &p, const PidCoeffs &c, float scale = 1.0f) {
        if (p.getKp() != c.kp * scale) p.setKp(c.kp * scale);
        if (p.getKi() != c.ki * scale) p.setKi(c.ki * scale);
        if (p.getKd() != c.kd * scale) p.setKd(c.kd * scale);
    }

    // ── Achsprojektion ────────────────────────────────────────────────────
    // Die IMU misst Roll und Pitch. Eine Motordiagonale liegt beim X-Rahmen
    // unter 45 Grad dazu, ihr Kippwinkel ist also die auf die Achsrichtung
    // projizierte Summe bzw. Differenz. Der Faktor 1/sqrt(2) macht daraus
    // wieder einen echten Winkel in Grad - ohne ihn waere der Wert um sqrt(2)
    // zu gross und alle daraus abgeleiteten Beiwerte entsprechend daneben.
    void axisProject(uint8_t ax, float roll, float pitch, float gr, float gp,
                     float &angle, float &rate) {
        switch (ax) {
            case AXIS_ROLL:  angle = roll;  rate = gr; break;
            case AXIS_PITCH: angle = pitch; rate = gp; break;
            // positiv = FL oben / BR unten
            case AXIS_FL_BR: angle = (pitch - roll) * DIAG_AXIS_GAIN;
                             rate  = (gp    - gr)   * DIAG_AXIS_GAIN; break;
            // positiv = FR oben / BL unten
            default:         angle = (pitch + roll) * DIAG_AXIS_GAIN;
                             rate  = (gp    + gr)   * DIAG_AXIS_GAIN; break;
        }
    }

    // Legt die Korrektur so auf die Mixer-Eingaenge um, dass genau die beiden
    // Motoren der Achse gegenlaeufig laufen. Siehe SharedState.h.
    void axisDrive(uint8_t ax, float corr, float &rollOut, float &pitchOut) {
        switch (ax) {
            case AXIS_ROLL:  rollOut  = corr; break;
            case AXIS_PITCH: pitchOut = corr; break;
            case AXIS_FL_BR: rollOut = -corr * 0.5f; pitchOut = corr * 0.5f; break;
            default:         rollOut =  corr * 0.5f; pitchOut = corr * 0.5f; break;
        }
    }

    // Die beiden Motoren, die auf dieser Achse gegeneinander arbeiten -
    // Spaltenauswahl fuer den Messschrieb. Indizes wie MotorMixer::getMotorUs:
    // 0=FL, 1=FR, 2=BL, 3=BR.
    void axisMotors(uint8_t ax, uint8_t &a, uint8_t &b) {
        switch (ax) {
            case AXIS_ROLL:  a = 0; b = 1; break;   // FL gegen FR
            case AXIS_PITCH: a = 0; b = 2; break;   // FL gegen BL
            case AXIS_FL_BR: a = 0; b = 3; break;   // FL gegen BR
            default:         a = 1; b = 2; break;   // FR gegen BL
        }
    }

    PIDController &axisPid(uint8_t ax) {
        switch (ax) {
            case AXIS_ROLL:  return s_pidRoll;
            case AXIS_PITCH: return s_pidPitch;
            default:         return s_pidDiag;
        }
    }

    // Beendet einen Bench-/Tune-Lauf und legt die Motoren still.
    void finish(uint16_t code) {
        s_running = false;
        s_abort   = code;
        recorder::stop();
        if (s_motors) s_motors->stopFast();
    }

    void startRun() {
        s_running    = true;
        s_abort      = ABORT_NONE;
        s_runStartMs = millis();
        s_angleTrips = 0;
        s_pidRoll.reset();
        s_pidPitch.reset();
        s_pidDiag.reset();
        if (s_motors) s_motors->setMaxUs((uint16_t)s_cmd.motorMaxUs);

        if (s_cmd.mode == MODE_TUNE) {
            s_tuner.begin(s_cmd.tune);
            TuneResult blank;
            resultWrite(blank);
        }
        recorder::start(0);
    }
}

void begin(IMU &imu, MotorMixer &motors) {
    s_imu    = &imu;
    s_motors = &motors;

    // Zwingend VOR jedem anderen Aufruf: setKp/reset/enableIntegral loggen
    // sonst per LOGGER_*, und der dahinterliegende logBuf ist EIN globaler
    // Puffer fuer beide Kerne (src/myLogger.cpp). Eine Ausgabe von hier
    // wuerde eine gleichzeitig laufende von Kern 0 zerstoeren.
    s_pidRoll.setQuiet(true);
    s_pidPitch.setQuiet(true);
    s_pidDiag.setQuiet(true);

    s_pidRoll.reset();
    s_pidPitch.reset();
    s_pidDiag.reset();
    recorder::begin();

    s_nextUs = micros();
    s_lastUs = s_nextUs;
    s_rateMs = millis();
}

void step() {
    // ── 1. Not-Aus zuerst, vor allem anderen ───────────────────────────
    if (g_estop) {
        if (s_motors) s_motors->stopFast();
        if (s_running) finish(ABORT_ESTOP);
        g_beatC1 = g_beatC1 + 1;
        return;
    }

    // Debug-Schalter aus 'stats -hang': laesst diesen Kern absichtlich
    // stehen, damit der Watchdog auf Kern 0 nachweisbar anschlaegt.
    // Bewusst OHNE Herzschlag - genau das soll Kern 0 bemerken.
    if (g_hangTest) {
        while (g_hangTest) tight_loop_contents();
        s_nextUs = micros();
    }

    // ── 2. Takt halten (selbstkorrigierend, driftfrei) ─────────────────
    s_nextUs += ATTITUDE_PERIOD_US;
    while ((int32_t)(micros() - s_nextUs) < 0) tight_loop_contents();
    if ((int32_t)(micros() - s_nextUs) > (int32_t)ATTITUDE_PERIOD_US) {
        ++s_overruns;
        s_nextUs = micros();   // nicht aufzuholen versuchen
    }

    uint32_t nowUs   = micros();
    uint32_t cycleUs = nowUs - s_lastUs;
    s_lastUs = nowUs;

    // ── 3. Neue Vorgaben holen (nie blockierend) ───────────────────────
    if (cmdFetch(s_cmd)) {
        applyCoeffs(s_pidRoll,  s_cmd.roll);
        applyCoeffs(s_pidPitch, s_cmd.pitch);
        applyCoeffs(s_pidDiag,  s_cmd.roll, 1.0f / DIAG_AXIS_GAIN);
        s_pidRoll.enableIntegral(s_cmd.integralOn);
        s_pidPitch.enableIntegral(s_cmd.integralOn);
        s_pidDiag.enableIntegral(s_cmd.integralOn);

        if (s_cmd.runId != s_lastRunId) {
            s_lastRunId = s_cmd.runId;
            if (s_cmd.mode == MODE_BENCH || s_cmd.mode == MODE_TUNE) startRun();
        }
        if (s_cmd.mode != s_lastMode) {
            s_lastMode = s_cmd.mode;
            if (s_cmd.mode == MODE_IDLE || s_cmd.mode == MODE_FLIGHT) {
                if (s_running) finish(ABORT_DISARM);
                if (s_motors) s_motors->setMaxUs(ESC_MAX_US);
            }
        }
    }

    // ── 4. Herzschlag von Kern 0 pruefen ───────────────────────────────
    static uint32_t seenBeatC0 = 0;
    static uint32_t seenBeatMs = 0;
    uint32_t b0 = g_beatC0;
    uint32_t nowMs = millis();
    if (b0 != seenBeatC0) { seenBeatC0 = b0; seenBeatMs = nowMs; }
    else if (seenBeatMs && (nowMs - seenBeatMs) > CORE0_WATCHDOG_MS) {
        if (s_motors) s_motors->stopFast();
        if (s_running) finish(ABORT_C0);
        g_beatC1 = g_beatC1 + 1;
        return;
    }

    // ── 5. IMU lesen ───────────────────────────────────────────────────
    bool ok = s_imu && s_imu->update(nowUs);
    float roll  = ok ? s_imu->getRoll()  : 0.0f;
    float pitch = ok ? s_imu->getPitch() : 0.0f;
    float gr    = ok ? s_imu->getGyroRoll()  : 0.0f;
    float gp    = ok ? s_imu->getGyroPitch() : 0.0f;

    float dt = cycleUs * 1e-6f;
    if (dt <= 0.0f || dt > 0.1f) dt = 1.0f / ATTITUDE_RATE_HZ;

    float rollOut = 0.0f, pitchOut = 0.0f;
    uint16_t thr = ESC_MIN_US;

    if (!ok) {
        if (s_motors) s_motors->stopFast();
        if (s_running) finish(ABORT_IMU);
    }
    else if (!s_cmd.armed || s_cmd.mode == MODE_IDLE) {
        if (s_motors) s_motors->stopFast();
        if (s_running) finish(ABORT_DISARM);
    }
    else if (s_cmd.mode == MODE_FLIGHT) {
        thr      = (uint16_t)s_cmd.throttleUs;
        rollOut  = s_pidRoll.computeWithRate(s_cmd.targetRoll,   roll,  gr, dt);
        pitchOut = s_pidPitch.computeWithRate(s_cmd.targetPitch, pitch, gp, dt);
        if (s_motors) s_motors->mix(thr, rollOut, pitchOut, 0.0f);
    }
    else if (s_running) {
        // ── Pruefstand / Autotune: genau EINE Achse ────────────────────
        // Achse ist entweder eine Flugachse (roll/pitch) oder eine
        // Motordiagonale; beides laeuft ueber dieselbe Projektion.
        const uint8_t ax = s_cmd.axis;
        float angle, rate;
        axisProject(ax, roll, pitch, gr, gp, angle, rate);

        float limit  = (s_cmd.mode == MODE_TUNE) ? s_cmd.tune.limitDeg
                                                 : s_cmd.bench.limitDeg;
        uint32_t tmo = (s_cmd.mode == MODE_TUNE) ? s_cmd.tune.timeoutMs
                                                 : s_cmd.bench.timeoutMs;

        // Drei Zyklen in Folge ueber der Grenze, damit ein einzelner
        // Ausreisser den Lauf nicht abbricht.
        if (fabsf(angle) > limit) {
            if (++s_angleTrips >= 3) { finish(ABORT_ANGLE); }
        } else s_angleTrips = 0;

        if (s_running && (millis() - s_runStartMs) > tmo) finish(ABORT_TIMEOUT);

        if (s_running) {
            thr = (uint16_t)s_cmd.throttleUs;
            float corr;

            PIDController &p = axisPid(ax);

            if (s_cmd.mode == MODE_TUNE) {
                corr = s_tuner.step(nowUs, angle);
                if (s_tuner.isDone()) {
                    TuneResult r = s_tuner.result();
                    r.axis = ax;
                    resultWrite(r);
                    finish(s_tuner.abortCode());
                    corr = 0.0f;
                }
            } else {
                // Sollwert ist auf jeder Achse die Waagerechte. Fuer die
                // Diagonalen gibt es keinen eigenen Sollwert: die Wippe soll
                // dort ebenso ausgeglichen stehen.
                float target = (ax == AXIS_PITCH) ? s_cmd.targetPitch
                                                  : s_cmd.targetRoll;
                corr = p.computeWithRate(target, angle, rate, dt);
            }

            axisDrive(ax, corr, rollOut, pitchOut);

            if (s_motors && s_running)
                s_motors->mix(thr, rollOut, pitchOut, 0.0f);

            // Aufzeichnen: die beiden Motoren, die auf dieser Achse
            // gegeneinander arbeiten.
            uint8_t iA, iB;
            axisMotors(ax, iA, iB);
            uint16_t m0 = s_motors ? s_motors->getMotorUs(iA) : 0;
            uint16_t m1 = s_motors ? s_motors->getMotorUs(iB) : 0;
            recorder::push(nowUs, angle, rate, corr,
                           s_cmd.mode == MODE_TUNE ? 0.0f : p.getP(),
                           s_cmd.mode == MODE_TUNE ? 0.0f : p.getI(),
                           s_cmd.mode == MODE_TUNE ? 0.0f : p.getD(),
                           m0, m1);
        }
    }
    else if (s_motors) {
        s_motors->stopFast();
    }

    // ── 6. Rate messen und Telemetrie veroeffentlichen ─────────────────
    if (cycleUs > s_maxLoopUs) s_maxLoopUs = cycleUs;
    ++s_rateCount;
    if ((uint32_t)(nowMs - s_rateMs) >= 1000u) {
        s_loopHz    = s_rateCount;
        s_rateCount = 0;
        s_rateMs    = nowMs;
        // Den Spitzenwert der abgelaufenen Sekunde festhalten und erst dann
        // neu messen. Ohne das Halten zeigte 'stats' fast immer den Wert
        // eines gerade erst begonnenen Fensters, also nahezu nichts.
        s_maxHeld   = s_maxLoopUs;
        s_maxLoopUs = 0;
    }

    CoreTlm t;
    t.us        = nowUs;
    t.roll      = roll;
    t.pitch     = pitch;
    t.gyroRoll  = gr;
    t.gyroPitch = gp;
    t.rollOut   = rollOut;
    t.pitchOut  = pitchOut;
    // Einzelbeitraege des Reglers, der gerade arbeitet: am Pruefstand der
    // Regler der aktiven Achse, sonst Roll. Vorher stand hier immer s_pidRoll,
    // was auf einem Pitch-Prueflauf die falsche Achse zeigte.
    PIDController &tp = (s_running && s_cmd.mode == MODE_BENCH)
                      ? axisPid(s_cmd.axis) : s_pidRoll;
    t.rollP     = tp.getP();
    t.rollI     = tp.getI();
    t.rollD     = tp.getD();
    for (uint8_t i = 0; i < 4; ++i) t.m[i] = s_motors ? s_motors->getMotorUs(i) : 0;
    t.loopHz    = s_loopHz;
    t.maxLoopUs = s_maxHeld;
    t.overruns  = s_overruns;
    t.abortCode = s_abort;
    t.imuReady  = ok;
    t.running   = s_running;
    tlmWrite(t);

    g_beatC1 = g_beatC1 + 1;
}

} // namespace attitude
