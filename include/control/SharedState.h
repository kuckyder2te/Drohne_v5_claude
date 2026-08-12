#pragma once

#include <Arduino.h>
#include <pico/mutex.h>
#include <hardware/sync.h>
#include "config.h"
#include "storage/Settings.h"   // PidCoeffs

// ── Datenaustausch zwischen den beiden Kernen ──────────────────────────
//
// Kern 0 (NormalMode::loop): Ultraschall, Batterie, CLI, Hoehen-PID, Logging.
// Kern 1 (AttitudeLoop):     IMU, Roll/Pitch-PID, Motor-Mixing @ATTITUDE_RATE_HZ.
//
// Der RP2040 ist ein Cortex-M0+ ohne LDREX/STREX. __atomic_* loest GCC hier
// ueber eine Interrupt-Sperre auf - das schuetzt gegen ISRs auf demselben
// Kern, NICHT gegen den anderen Kern. Nutzbar ist deshalb nur:
//   - ausgerichtete 32-Bit-volatile-Zugriffe (auf dem RP2040-Bus atomar),
//   - Mutex fuer alles Groessere.
// Beides kommt hier vor, jeweils dort, wo es passt.
namespace shared {

// Betriebsart des Lageregelkreises auf Kern 1.
enum : uint8_t {
    MODE_IDLE   = 0,   // Motoren auf ESC_MIN_US, nichts geregelt
    MODE_FLIGHT = 1,   // normaler Flug: beide Achsen, Throttle vom Hoehen-PID
    MODE_BENCH  = 2,   // Pruefstand: feste Throttle, genau eine Achse
    MODE_TUNE   = 3    // Relay-Feedback-Autotune auf genau einer Achse
};

// Warum ein Lauf beendet wurde. 0 = laeuft/sauber beendet.
enum : uint16_t {
    ABORT_NONE     = 0,
    ABORT_ANGLE    = 1,   // Winkelgrenze ueberschritten
    ABORT_TIMEOUT  = 2,
    ABORT_ESTOP    = 3,   // Not-Aus 'd'
    ABORT_C0       = 4,   // Kern 0 antwortet nicht mehr
    ABORT_DISARM   = 5,
    ABORT_NOSWITCH = 6,   // Relais schaltet nicht -> keine Schwingung
    ABORT_IMU      = 7,
    ABORT_DONE     = 8    // Autotune regulaer fertig
};

// Achsen fuer BENCH/TUNE. Roll und Pitch sind die Flugachsen; die beiden
// Diagonalen sind die physischen Motorachsen des X-Rahmens - benannt nach den
// beiden Motoren, die die Wippe antreiben, waehrend die anderen beiden auf der
// Wippenstange liegen und unberuehrt bleiben.
//
//   AXIS_FL_BR   Stange auf FR-BL, angetrieben von FL gegen BR
//   AXIS_FR_BL   Stange auf FL-BR, angetrieben von FR gegen BL
//
// Der Mixer braucht dafuer keine Sonderbehandlung: eine Korrektur c, als
// rollOut=-c/2 und pitchOut=+c/2 eingespeist, ergibt FL=t+c, BR=t-c und laesst
// FR und BL exakt auf t stehen (analog mit rollOut=+c/2 fuer die andere
// Diagonale). Die Halbierung haelt c in derselben Einheit wie bei Roll/Pitch,
// naemlich der tatsaechlichen Abweichung eines Motors in us - nur so bedeutet
// die Relaisamplitude 'h' auf allen vier Achsen dasselbe.
constexpr uint8_t AXIS_ROLL  = 0;
constexpr uint8_t AXIS_PITCH = 1;
constexpr uint8_t AXIS_FL_BR = 2;
constexpr uint8_t AXIS_FR_BL = 3;

inline bool axisIsDiagonal(uint8_t ax) { return ax == AXIS_FL_BR || ax == AXIS_FR_BL; }

struct BenchCfg {
    float    limitDeg  = BENCH_LIMIT_DEG;
    uint32_t timeoutMs = BENCH_TIMEOUT_S * 1000UL;
};

struct TuneCfg {
    float    h        = TUNE_H_US;        // Relais-Amplitude in us
    float    eps      = TUNE_EPS_DEG;     // Hysterese in Grad
    uint8_t  cycles   = TUNE_CYCLES;      // auszuwertende Perioden
    uint8_t  warmup   = TUNE_WARMUP_CYCLES;
    float    limitDeg = TUNE_LIMIT_DEG;
    uint32_t timeoutMs = TUNE_TIMEOUT_S * 1000UL;
};

// ── Kern 0 -> Kern 1 ───────────────────────────────────────────────────
// Selten geaendert (CLI-Kommando, Hoehen-PID @20 Hz). Mutex-geschuetzt,
// aber Kern 1 wartet NIE darauf: es prueft lock-frei g_cmdGen und holt sich
// die neue Fassung nur, wenn mutex_try_enter() sofort gelingt. Andernfalls
// regelt es diese Runde mit der alten Kopie weiter und versucht es 2,5 ms
// spaeter erneut. So kann ein CLI-Kommando den Regeltakt nicht stoeren.
struct CoreCmd {
    bool      armed        = false;
    uint8_t   mode         = MODE_IDLE;
    uint8_t   axis         = AXIS_ROLL;   // aktive Achse in BENCH/TUNE
    float     throttleUs   = ESC_MIN_US;  // Hoehen-PID (FLIGHT) bzw. fest (BENCH/TUNE)
    float     targetRoll   = TARGET_ROLL_DEG;
    float     targetPitch  = TARGET_PITCH_DEG;
    float     motorMaxUs   = ESC_MAX_US;
    bool      integralOn   = false;       // Liftoff-Gate, kommt vom Hoehen-PID
    PidCoeffs roll{PID_KP_ROLL, PID_KI_ROLL, PID_KD_ROLL};
    PidCoeffs pitch{PID_KP_PITCH, PID_KI_PITCH, PID_KD_PITCH};
    BenchCfg  bench;
    TuneCfg   tune;
    uint32_t  runId = 0;   // hochzaehlen startet einen BENCH-/TUNE-Lauf neu
};

// ── Kern 1 -> Kern 0 ───────────────────────────────────────────────────
// Jeden Zyklus geschrieben. Seqlock: der Schreiber blockiert nie, der Leser
// wiederholt, falls waehrenddessen geschrieben wurde.
struct CoreTlm {
    uint32_t us          = 0;
    float    roll        = 0.0f;   // Grad
    float    pitch       = 0.0f;
    float    gyroRoll    = 0.0f;   // Grad/s
    float    gyroPitch   = 0.0f;
    float    rollOut     = 0.0f;   // us Korrektur
    float    pitchOut    = 0.0f;
    float    rollP       = 0.0f;   // Einzelbeitraege der aktiven Achse
    float    rollI       = 0.0f;
    float    rollD       = 0.0f;
    uint16_t m[4]        = {ESC_MIN_US, ESC_MIN_US, ESC_MIN_US, ESC_MIN_US};
    uint32_t loopHz      = 0;
    uint32_t maxLoopUs   = 0;
    uint32_t overruns    = 0;
    uint16_t abortCode   = ABORT_NONE;
    bool     imuReady    = false;
    bool     running     = false;  // BENCH/TUNE laeuft gerade
};

// Ergebnis eines Autotune-Laufs. Kern 1 fuellt es, Kern 0 rechnet daraus die
// Koeffizienten - die Umrechnung braucht kein Echtzeitverhalten.
struct TuneResult {
    bool     valid    = false;
    uint8_t  axis     = AXIS_ROLL;
    float    h        = 0.0f;
    float    eps      = 0.0f;
    float    amp      = 0.0f;   // Grad, Mittelwert der Halbamplituden
    float    tu       = 0.0f;   // s, Mittelwert der Periodendauern
    float    spread   = 0.0f;   // relative Streuung von tu (0..1)
    uint8_t  n        = 0;      // ausgewertete Perioden
    uint16_t abortCode = ABORT_NONE;
};

// ── Lock-freie Einzelwerte ─────────────────────────────────────────────
// 32 Bit, ausgerichtet -> einzelne Lade-/Speicherzugriffe sind auf dem
// RP2040-Bus atomar. Kein Mutex, damit der Not-Aus keinen warten kann.
extern volatile bool     g_estop;       // Kern 0 -> Kern 1, sofort wirksam
extern volatile uint32_t g_beatC0;      // Herzschlag Kern 0
extern volatile uint32_t g_beatC1;      // Herzschlag Kern 1
extern volatile bool     g_core0Ready;  // Kern 1 wartet damit auf setup()
extern volatile bool     g_hangTest;    // Debug: laesst Kern 1 absichtlich haengen

// ── Befehlsblock ───────────────────────────────────────────────────────
void    cmdWrite(const CoreCmd &c);      // Kern 0: veroeffentlichen
bool    cmdFetch(CoreCmd &out);          // Kern 1: holen, nur wenn neu und frei
CoreCmd cmdRead();                        // Kern 0: aktuelle Fassung lesen

// ── Telemetrie ─────────────────────────────────────────────────────────
void tlmWrite(const CoreTlm &t);          // Kern 1
void tlmRead(CoreTlm &out);               // Kern 0

// ── Autotune-Ergebnis ──────────────────────────────────────────────────
void resultWrite(const TuneResult &r);
void resultRead(TuneResult &out);

} // namespace shared
