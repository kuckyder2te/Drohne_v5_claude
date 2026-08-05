#include "control/Recorder.h"
#include "control/SharedState.h"

#include <Arduino.h>
#include <hardware/sync.h>

namespace recorder {

namespace {
    // Statisch in der .bss, kein malloc: so steht die Puffergroesse direkt in
    // der Build-Ausgabe und laesst sich ohne Laufzeittest nachpruefen.
    Sample s_buf[REC_CAPACITY];

    volatile uint16_t s_count  = 0;
    volatile bool     s_active = false;
    uint32_t          s_endMs  = 0;
    uint8_t           s_skip   = 0;

    int16_t clamp16(float v) {
        if (v >  32767.0f) return  32767;
        if (v < -32768.0f) return -32768;
        return (int16_t)v;
    }
}

void begin() {
    s_count  = 0;
    s_active = false;
}

void start(uint32_t durationMs) {
    s_count  = 0;
    s_skip   = 0;
    s_endMs  = durationMs ? (millis() + durationMs) : 0;
    __dmb();
    s_active = true;
}

void stop() {
    s_active = false;
    __dmb();
}

bool isActive() { return s_active; }

uint16_t count()    { return s_count; }
uint16_t capacity() { return REC_CAPACITY; }

void push(uint32_t us, float angleDeg, float rateDps, float out,
          float p, float i, float d, uint16_t m0, uint16_t m1)
{
    if (!s_active) return;

    if (++s_skip < REC_DECIMATION) return;
    s_skip = 0;

    uint16_t n = s_count;
    if (n >= REC_CAPACITY) { s_active = false; return; }

    if (s_endMs && (int32_t)(millis() - s_endMs) >= 0) { s_active = false; return; }

    Sample &x = s_buf[n];
    x.us         = us;
    x.angle_cdeg = clamp16(angleDeg * 100.0f);
    // 0,1 Grad/s statt 0,01: bei Gyro-Bereich 500 dps waeren +-327 Grad/s
    // sonst zu wenig und die Relais-Anregung wuerde die Spalte saettigen.
    x.rate_cdps  = clamp16(rateDps * 10.0f);
    x.out_us     = clamp16(out);
    x.p_us       = clamp16(p);
    x.i_us       = clamp16(i);
    x.d_us       = clamp16(d);
    x.m0         = m0;
    x.m1         = m1;

    __dmb();          // Sample steht, bevor der Zaehler es freigibt
    s_count = n + 1;
}

bool dump(Stream &out, uint16_t lastN) {
    if (s_active) return false;

    uint16_t n = s_count;
    uint16_t from = 0;
    if (lastN && lastN < n) from = n - lastN;

    out.println(F("us,angle_deg,rate_dps,out_us,p_us,i_us,d_us,m0,m1"));

    for (uint16_t k = from; k < n; ++k) {
        const Sample &x = s_buf[k];
        out.print(x.us);                       out.print(',');
        out.print(x.angle_cdeg / 100.0f, 2);   out.print(',');
        out.print(x.rate_cdps  / 10.0f,  1);   out.print(',');
        out.print(x.out_us);                   out.print(',');
        out.print(x.p_us);                     out.print(',');
        out.print(x.i_us);                     out.print(',');
        out.print(x.d_us);                     out.print(',');
        out.print(x.m0);                       out.print(',');
        out.println(x.m1);

        // Der Auszug haelt Kern 0 sekundenlang in diesem Aufruf fest. Ohne
        // das hier liefe der Herzschlag ab und Kern 1 wuerde die Motoren
        // abschalten - deswegen ist dump() ausserdem nur disarmiert erlaubt.
        shared::g_beatC0 = shared::g_beatC0 + 1;
    }

    out.print(F("#end "));
    out.print(n - from);
    out.println(F(" samples"));
    return true;
}

} // namespace recorder
