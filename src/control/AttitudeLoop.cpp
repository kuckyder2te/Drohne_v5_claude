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

    void applyCoeffs(PIDController &p, const PidCoeffs &c) {
        if (p.getKp() != c.kp) p.setKp(c.kp);
        if (p.getKi() != c.ki) p.setKi(c.ki);
        if (p.getKd() != c.kd) p.setKd(c.kd);
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

    s_pidRoll.reset();
    s_pidPitch.reset();
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
        s_pidRoll.enableIntegral(s_cmd.integralOn);
        s_pidPitch.enableIntegral(s_cmd.integralOn);

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
        bool  isRoll = (s_cmd.axis == AXIS_ROLL);
        float angle  = isRoll ? roll : pitch;
        float rate   = isRoll ? gr   : gp;
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

            if (s_cmd.mode == MODE_TUNE) {
                corr = s_tuner.step(nowUs, angle);
                if (s_tuner.isDone()) {
                    TuneResult r = s_tuner.result();
                    r.axis = s_cmd.axis;
                    resultWrite(r);
                    finish(s_tuner.abortCode());
                    corr = 0.0f;
                }
            } else {
                corr = isRoll
                     ? s_pidRoll.computeWithRate(s_cmd.targetRoll,   angle, rate, dt)
                     : s_pidPitch.computeWithRate(s_cmd.targetPitch, angle, rate, dt);
            }

            if (isRoll) rollOut = corr; else pitchOut = corr;

            if (s_motors && s_running)
                s_motors->mix(thr, rollOut, pitchOut, 0.0f);

            // Aufzeichnen: fuer Roll treibt die Korrektur FL(0) gegen FR(1)
            // auseinander, fuer Pitch FL(0) gegen BL(2).
            PIDController &p = isRoll ? s_pidRoll : s_pidPitch;
            uint16_t m0 = s_motors ? s_motors->getMotorUs(0) : 0;
            uint16_t m1 = s_motors ? s_motors->getMotorUs(isRoll ? 1 : 2) : 0;
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
    t.rollP     = s_pidRoll.getP();
    t.rollI     = s_pidRoll.getI();
    t.rollD     = s_pidRoll.getD();
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
