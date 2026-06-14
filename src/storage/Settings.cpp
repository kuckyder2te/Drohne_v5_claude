#include "storage/Settings.h"
#include "myLogger.h"
#include "config.h"

void Settings::begin() {
    EEPROM.begin(EEPROM_SIZE);
    LOG("[EEPROM] Initialisiert");
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
    LOG("[EEPROM] Gespeichert");
    LOG_FMT("[EEPROM] Kp=%.4f",kp); 
    LOG_FMT("[EEPROM] Ki=%.4f",ki);  
    LOG_FMT("[EEPROM] Kd=%.4f",kd);  
}

bool Settings::load(float& kp, float& ki, float& kd) {
    // Pruefen ob gueltige Daten vorhanden
    uint8_t marker = EEPROM.read(EEPROM_VALID_ADDR);
    if (marker != EEPROM_VALID_VAL) {
        LOG("[EEPROM] Keine gueltigen Daten - Standardwerte");
        return false;
    }
    kp = _readFloat(EEPROM_ADDR_KP);
    ki = _readFloat(EEPROM_ADDR_KI);
    kd = _readFloat(EEPROM_ADDR_KD);
    LOG("[EEPROM] Geladen");
    LOG_FMT("[EEPROM] Kp= %.4f",kp); 
    LOG_FMT("[EEPROM] Ki= %.4f",ki); 
    LOG_FMT("[EEPROM] Kd= %.4f",kd); 
    return true;
}

void Settings::reset() {
    EEPROM.write(EEPROM_VALID_ADDR, 0x00);
    EEPROM.commit();
    LOG("[EEPROM] Zurueckgesetzt");
}