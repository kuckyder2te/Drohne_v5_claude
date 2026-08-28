#pragma once

#include <Arduino.h>

#define ULTRASONIC_MAX_CM    300.0f  // max 3m
#define ULTRASONIC_MIN_CM      2.0f  // min 2cm
#define ULTRASONIC_FILTER_SIZE   5   // Mittelwertfilter (pro Sensor!)
#define ULTRASONIC_COUNT         2   // beide nach unten gerichtet

// Mindestabstand zwischen zwei Triggern - egal wie oft der Aufrufer update()
// ruft. Frueher stand dafuer ein delay(30) am Ende von update(); das hat den
// Kern 0 blockiert, obwohl NormalMode::loop() ohnehin nur alle ULTRA_UPDATE_MS
// (50 ms) misst. Als Sperre in der Klasse kostet es nichts und schuetzt auch
// Aufrufer ohne eigene Kadenz (src/tools/test_ultrasonic).
#define ULTRASONIC_MIN_PING_MS  40

// Ein Sensor faellt aus der Minimum-Bildung, wenn sein letzter GUELTIGER Wert
// aelter ist. 250 ms = 2,5 x der Wechselperiode (2 * ULTRA_UPDATE_MS = 100 ms):
// zwei ausgefallene Pings werden ueberbrueckt, ein toter Sensor fliegt raus.
//
// Diese Grenze ist bei Minimum-Fusion sicherheitsrelevant: ohne sie wuerde ein
// abgestuerzter Sensor seinen letzten (womoeglich kleinen) Wert fuer immer im
// Minimum halten und der Hoehenregler dagegen ansteigen.
#define ULTRASONIC_STALE_MS    250

// Zwei HC-SR04, beide nach unten. Gemessen wird pro update() genau EIN Sensor,
// im Wechsel - zwei Messungen in einem Aufruf waeren bis zu 50 ms pulseIn und
// wuerden Kern 0 doppelt so lange blockieren wie bisher. Je Sensor also 100 ms
// Abtastung, die fusionierte Hoehe wird trotzdem alle 50 ms neu gebildet.
//
// Fusion ist das MINIMUM der frischen, gueltigen Kanaele: ueber unebenem Boden
// haelt die Drohne damit Abstand zum naechstliegenden Hindernis, statt darueber
// abzusinken. Ein automatischer Ausreisser-Ausschluss existiert bewusst nicht -
// bei zwei Sensoren ist nicht entscheidbar, welcher luegt. Stattdessen wird die
// Abweichung ueber getSpreadCm() sichtbar gemacht ('getDistance', Statuszeile).
class Ultrasonic {
public:
    void begin();
    void update();   // misst GENAU EINEN Sensor, dann Wechsel

    // Fusionierter Wert. Signaturen unveraendert, damit FlightController und
    // NormalMode unberuehrt bleiben.
    float getAltitudeCm() const { return _altitudeCm; }
    bool  isValid()       const;   // wird live geprueft, nicht zwischengespeichert

    // Einzelne Sensoren (0 = TRIG1/ECHO1, 1 = TRIG2/ECHO2)
    float getAltitudeCm(uint8_t i) const;
    bool  isValid(uint8_t i)       const;  // frisch (siehe ULTRASONIC_STALE_MS)
    float getSpreadCm()            const;  // |S0-S1|, -1 wenn nicht beide frisch

private:
    struct Chan {
        uint8_t  trigPin     = 0;
        uint8_t  echoPin     = 0;
        float    altitudeCm  = 0.0f;

        // Einziges Gueltigkeitskriterium: Zeitpunkt des letzten GUELTIGEN
        // Messwerts, 0 = noch nie gemessen. Ein zusaetzliches "letzte Messung
        // war gut"-Flag waere hier falsch: es wuerde einen Kanal schon nach
        // EINEM Fehlping aus der Fusion werfen, und die fusionierte Hoehe
        // spraenge bei jedem Aussetzer auf den anderen Sensor und zurueck -
        // ein Stufensprung direkt in den Hoehen-PID.
        uint32_t lastValidMs = 0;

        // Ringpuffer Filter - pro Kanal. Ein gemeinsamer Puffer wuerde die
        // Messwerte zweier verschieden montierter Sensoren zu einem Mittelwert
        // verruehren, den es real nirgends gibt.
        float    filterBuf[ULTRASONIC_FILTER_SIZE] = {0};
        uint8_t  filterIdx   = 0;
        bool     filterFull  = false;
    };

    Chan     _ch[ULTRASONIC_COUNT];
    uint8_t  _current       = 0;
    uint32_t _lastTriggerMs = 0;

    // Fusionierter Wert (Minimum). Bleibt absichtlich stehen, wenn kein Kanal
    // mehr frisch ist - siehe _fuse().
    float _altitudeCm = 0.0f;

    void  _trigger(uint8_t trigPin);
    float _measureCm(uint8_t echoPin);
    float _applyFilter(Chan &c, float newValue);
    void  _fuse();
};
