#pragma once

#include <Arduino.h>
#include "pins.h"
#include "config.h"   // ESC_MIN_US/ESC_MAX_US - kamen frueher transitiv ueber myLogger.h

class MotorMixer
{
public:
    // PWM aufsetzen, MIN ausgeben und die ESCs anschliessend ueber den
    // MOSFET an PIN_ESC_POWER zuschalten. autoPower=false laesst sie
    // stromlos - das brauchen die Testwerkzeuge, weil der ESC seine
    // Betriebsart aus dem Signal ableitet, das beim Einschalten anliegt
    // (MIN = Normalbetrieb, MAX = Kalibrierung).
    void begin(bool autoPower = true);

    // ── ESC-Stromversorgung (MOSFET, PIN_ESC_POWER) ────────────────────
    // powerOn() aendert die PWM absichtlich NICHT: was gerade ausgegeben
    // wird, ist die Betriebsart, in der der ESC hochlaeuft. Wer normalen
    // Betrieb will, sorgt vorher selbst fuer MIN (stop()/stopFast()).
    // Beides sind reine Registerzugriffe, ohne Log, von jedem Kern sicher.
    void powerOn();
    void powerOff();
    bool isPowered() const { return _powered; }

    void setThrottle(uint16_t throttle_us); // 1000–2000 µs
    void mix(uint16_t throttle, float roll, float pitch, float yaw);
    void setSingle(uint8_t motor, uint16_t throttle); // ← NEU: 1=FL 2=FR 3=BR 4=BL
    void stop();

    // Logfreier Not-Aus. stop() ruft LOGGER_NOTICE und schreibt damit in den
    // globalen logBuf - das darf weder Kern 1 noch der Not-Aus-Pfad, der
    // mitten in einer laufenden Ausgabe von Kern 0 zuschlagen kann.
    // Reine Registerschreibzugriffe, idempotent, von jedem Kern aus sicher.
    void stopFast();

    // Obergrenze fuer alle vier Kanaele. Am Pruefstand soll ein
    // fehlgeschlagener Regler die Motoren nicht bis 2000 us hochfahren.
    void setMaxUs(uint16_t us) { _maxUs = us; }
    uint16_t getMaxUs() const  { return _maxUs; }

    // Reihenfolge: 0=FL, 1=FR, 2=BL, 3=BR
    uint16_t getMotorUs(uint8_t i) const;

private:
    uint16_t _fl = 1000, _fr = 1000, _bl = 1000, _br = 1000;
    uint16_t _throttle_us = 1000;
    uint16_t _maxUs = ESC_MAX_US;
    bool _powered = false;

    void _writePWM(uint8_t pin, uint16_t us);
};
