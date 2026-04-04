#include "storage/Settings.h"
#include "config.h"

void Settings::begin() {
    EEPROM.begin(EEPROM_SIZE);
    Serial.println("[EEPROM] Initialisiert");
}

void Settings::_writeFloat(int addr, float val) {
    EEPROM.put(addr, val);
}

float Settings::_readFloat(int addr) {
    float val = 0.0f;
    EEPROM.get(addr, val);
    return val;
}

void Settings::save(float kp, float ki, float kd) {
    _writeFloat(EEPROM_ADDR_KP, kp);
    _writeFloat(EEPROM_ADDR_KI, ki);
    _writeFloat(EEPROM_ADDR_KD, kd);
    EEPROM.write(EEPROM_VALID_ADDR, EEPROM_VALID_VAL);
    EEPROM.commit();
    Serial.println("[EEPROM] Gespeichert");
    Serial.print("[EEPROM] Kp="); Serial.print(kp, 4);
    Serial.print(" Ki=");         Serial.print(ki, 4);
    Serial.print(" Kd=");         Serial.println(kd, 4);
}

bool Settings::load(float& kp, float& ki, float& kd) {
    // Prüfen ob gültige Daten vorhanden
    uint8_t marker = EEPROM.read(EEPROM_VALID_ADDR);
    if (marker != EEPROM_VALID_VAL) {
        Serial.println("[EEPROM] Keine gültigen Daten — Standardwerte");
        return false;
    }
    kp = _readFloat(EEPROM_ADDR_KP);
    ki = _readFloat(EEPROM_ADDR_KI);
    kd = _readFloat(EEPROM_ADDR_KD);
    Serial.println("[EEPROM] Geladen");
    Serial.print("[EEPROM] Kp="); Serial.print(kp, 4);
    Serial.print(" Ki=");         Serial.print(ki, 4);
    Serial.print(" Kd=");         Serial.println(kd, 4);
    return true;
}

void Settings::reset() {
    EEPROM.write(EEPROM_VALID_ADDR, 0x00);
    EEPROM.commit();
    Serial.println("[EEPROM] Zurückgesetzt");
}