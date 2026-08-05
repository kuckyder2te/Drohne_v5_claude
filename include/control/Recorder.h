#pragma once

#include <Arduino.h>
#include "config.h"

class Stream;

// ── Aufzeichnung des Lageregelkreises ──────────────────────────────────
//
// Ein Erzeuger (Kern 1, push()), ein Verbraucher (Kern 0, dump()). Es gibt
// bewusst kein Lock: der Verbraucher liest ausschliesslich, nachdem die
// Aufzeichnung beendet ist (isActive() == false). dump() prueft das.
//
// Ohne diese Aufzeichnung sind die Zahlen eines Autotune-Laufs nicht
// nachpruefbar - der CSV-Auszug ist der Beleg, dass die Grenzschwingung
// sauber war und nicht etwa der Anschlag der Wippe gemessen wurde.
namespace recorder {

// 20 Byte, 4-Byte-ausgerichtet.
struct Sample {
    uint32_t us;             //  4
    int16_t  angle_cdeg;     //  6  0,01 Grad   -> +-327 Grad
    int16_t  rate_cdps;      //  8  0,1 Grad/s  -> +-3276 Grad/s
    int16_t  out_us;         // 10  Reglerausgang bzw. Relaisstellung
    int16_t  p_us;           // 12  Einzelbeitraege
    int16_t  i_us;           // 14
    int16_t  d_us;           // 16
    uint16_t m0;             // 18  die beiden Motoren der aktiven Achse
    uint16_t m1;             // 20
};

void begin();

// Kern 1: startet eine Aufzeichnung. durationMs == 0 -> bis stop() oder
// bis der Puffer voll ist.
void start(uint32_t durationMs);
void stop();
bool isActive();

// Kern 1, einmal je Regelzyklus. Die Dezimierung auf REC_DECIMATION
// steckt hier drin, der Aufrufer muss nicht mitzaehlen.
void push(uint32_t us, float angleDeg, float rateDps, float out,
          float p, float i, float d, uint16_t m0, uint16_t m1);

uint16_t count();
uint16_t capacity();

// Kern 0: CSV auf den Shell-Kanal. lastN == 0 -> alles.
// Gibt false zurueck, wenn noch aufgezeichnet wird.
bool dump(Stream &out, uint16_t lastN);

} // namespace recorder
