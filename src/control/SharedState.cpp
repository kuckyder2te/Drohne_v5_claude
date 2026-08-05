#include "control/SharedState.h"

namespace shared {

volatile bool     g_estop      = false;
volatile uint32_t g_beatC0     = 0;
volatile uint32_t g_beatC1     = 0;
volatile bool     g_core0Ready = false;
volatile bool     g_hangTest   = false;

namespace {
    // Befehlsblock: Mutex + Generationszaehler.
    auto_init_mutex(s_cmdMutex);
    CoreCmd           s_cmd;
    volatile uint32_t s_cmdGen = 0;

    // Telemetrie: Seqlock. Ungerade Sequenznummer = Schreibvorgang laeuft.
    volatile uint32_t s_tlmSeq = 0;
    CoreTlm           s_tlm;

    // Autotune-Ergebnis: wird genau einmal pro Lauf geschrieben und danach
    // gelesen, hier reicht dieselbe Seqlock-Technik.
    volatile uint32_t s_resSeq = 0;
    TuneResult        s_res;
}

// ── Befehlsblock ───────────────────────────────────────────────────────

void cmdWrite(const CoreCmd &c) {
    mutex_enter_blocking(&s_cmdMutex);
    s_cmd = c;
    mutex_exit(&s_cmdMutex);
    __dmb();
    s_cmdGen = s_cmdGen + 1;   // erst NACH dem Freigeben: Kern 1 soll die
                               // neue Generation nur sehen, wenn es den
                               // Mutex auch bekommen kann.
}

CoreCmd cmdRead() {
    mutex_enter_blocking(&s_cmdMutex);
    CoreCmd copy = s_cmd;
    mutex_exit(&s_cmdMutex);
    return copy;
}

// Kern 1: nur holen, wenn es etwas Neues gibt UND der Mutex sofort frei ist.
// Rueckgabe false heisst "behalte deine bisherige Kopie" - nie blockieren.
bool cmdFetch(CoreCmd &out) {
    static uint32_t seen = 0xFFFFFFFFu;   // erzwingt das erste Holen

    uint32_t gen = s_cmdGen;
    if (gen == seen) return false;

    if (!mutex_try_enter(&s_cmdMutex, nullptr)) return false;
    out = s_cmd;
    mutex_exit(&s_cmdMutex);

    seen = gen;
    return true;
}

// ── Telemetrie (Seqlock) ───────────────────────────────────────────────

void tlmWrite(const CoreTlm &t) {
    s_tlmSeq = s_tlmSeq + 1;   // ungerade -> Schreibvorgang laeuft
    __dmb();
    s_tlm = t;
    __dmb();
    s_tlmSeq = s_tlmSeq + 1;   // wieder gerade
}

void tlmRead(CoreTlm &out) {
    // Ein Abbruch nach einigen Versuchen ist noetig, damit ein haengender
    // Kern 1 mit ungerader Sequenznummer den Leser nicht ewig festhaelt -
    // genau der Fall, den der Watchdog melden soll.
    for (int tries = 0; tries < 8; ++tries) {
        uint32_t s1 = s_tlmSeq;
        if (s1 & 1u) continue;
        __dmb();
        out = s_tlm;
        __dmb();
        if (s_tlmSeq == s1) return;
    }
    out = s_tlm;   // Notnagel: moeglicherweise inkonsistent, aber nie haengend
}

// ── Autotune-Ergebnis ──────────────────────────────────────────────────

void resultWrite(const TuneResult &r) {
    s_resSeq = s_resSeq + 1;
    __dmb();
    s_res = r;
    __dmb();
    s_resSeq = s_resSeq + 1;
}

void resultRead(TuneResult &out) {
    for (int tries = 0; tries < 8; ++tries) {
        uint32_t s1 = s_resSeq;
        if (s1 & 1u) continue;
        __dmb();
        out = s_res;
        __dmb();
        if (s_resSeq == s1) return;
    }
    out = s_res;
}

} // namespace shared
