#pragma once

#include <Arduino.h>
#include <EEPROM.h>

// Ein PID-Koeffizientensatz. Bewusst ein eigener Typ statt neun Floats in
// der Signatur - so lassen sich die Achsen beim Aufruf nicht vertauschen.
struct PidCoeffs {
    float kp;
    float ki;
    float kd;
};

// ── EEPROM-Layout ──────────────────────────────────────────
// Drei Achsen a drei float (12 Byte), danach der Gueltigkeitsmarker.
// Einzige Wahrheit fuer diese Adressen - frueher standen sie zusaetzlich
// in config.h, was beim Aendern still auseinanderlaufen konnte.
#define EEPROM_ADDR_HEIGHT 0    // kp @ +0, ki @ +4, kd @ +8
#define EEPROM_ADDR_ROLL   12
#define EEPROM_ADDR_PITCH  24
#define EEPROM_VALID_ADDR  36
#define EEPROM_SIZE        64

// Der Marker ist zugleich die Layout-Version. Er wurde von 0xAB auf 0xAC
// erhoeht, als Roll und Pitch dazukamen: ein EEPROM aus der Zeit davor
// faellt dadurch sauber auf die Standardwerte zurueck, statt Bytes des
// alten Layouts als Roll-/Pitch-Koeffizienten zu interpretieren.
#define EEPROM_VALID_VAL   0xAC

class Settings {
public:
    void begin();

    // Speichert bzw. laedt alle drei Regler gemeinsam - es gibt nur einen
    // Gueltigkeitsmarker fuer den ganzen Block, ein Teil-Speichern waere
    // also nicht darstellbar.
    void save(const PidCoeffs& height, const PidCoeffs& roll, const PidCoeffs& pitch);
    bool load(PidCoeffs& height, PidCoeffs& roll, PidCoeffs& pitch);

    void reset();  // Marker loeschen -> beim naechsten Start Standardwerte

private:
    void _writeCoeffs(int addr, const PidCoeffs& c);
    void _readCoeffs(int addr, PidCoeffs& c);
};
