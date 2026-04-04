#pragma once

#include <Arduino.h>
#include <EEPROM.h>

// EEPROM-Adressen
#define EEPROM_ADDR_KP    0
#define EEPROM_ADDR_KI    4
#define EEPROM_ADDR_KD    8
#define EEPROM_VALID_ADDR 12   // Validierungs-Marker
#define EEPROM_VALID_VAL  0xAB // Bekannter Wert → Daten gültig
#define EEPROM_SIZE       16

class Settings {
public:
    void begin();
    void save(float kp, float ki, float kd);
    bool load(float& kp, float& ki, float& kd);
    void reset();  // Auf Standardwerte zurücksetzen

private:
    void _writeFloat(int addr, float val);
    float _readFloat(int addr);
};