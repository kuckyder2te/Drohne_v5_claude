#include "storage/Settings.h"
#include "myLogger.h"
#include "config.h"

void Settings::begin() {
    EEPROM.begin(EEPROM_SIZE);
    LOGGER_NOTICE("[EEPROM] Initialisiert");
}

void Settings::_writeCoeffs(int addr, const PidCoeffs& c) {
    EEPROM.put(addr + 0, c.kp);
    EEPROM.put(addr + 4, c.ki);
    EEPROM.put(addr + 8, c.kd);
}

void Settings::_readCoeffs(int addr, PidCoeffs& c) {
    EEPROM.get(addr + 0, c.kp);
    EEPROM.get(addr + 4, c.ki);
    EEPROM.get(addr + 8, c.kd);
}

void Settings::save(const PidCoeffs& height, const PidCoeffs& roll, const PidCoeffs& pitch) {
    _writeCoeffs(EEPROM_ADDR_HEIGHT, height);
    _writeCoeffs(EEPROM_ADDR_ROLL,   roll);
    _writeCoeffs(EEPROM_ADDR_PITCH,  pitch);
    EEPROM.write(EEPROM_VALID_ADDR, EEPROM_VALID_VAL);
    EEPROM.commit();
    LOGGER_NOTICE("[EEPROM] Gespeichert");
    LOGGER_NOTICE_FMT("[EEPROM] height Kp=%.4f Ki=%.4f Kd=%.4f", height.kp, height.ki, height.kd);
    LOGGER_NOTICE_FMT("[EEPROM] roll   Kp=%.4f Ki=%.4f Kd=%.4f", roll.kp,   roll.ki,   roll.kd);
    LOGGER_NOTICE_FMT("[EEPROM] pitch  Kp=%.4f Ki=%.4f Kd=%.4f", pitch.kp,  pitch.ki,  pitch.kd);
}

bool Settings::load(PidCoeffs& height, PidCoeffs& roll, PidCoeffs& pitch) {
    // Pruefen ob gueltige Daten im aktuellen Layout vorhanden sind
    uint8_t marker = EEPROM.read(EEPROM_VALID_ADDR);
    if (marker != EEPROM_VALID_VAL) {
        LOGGER_NOTICE("[EEPROM] Keine gueltigen Daten - Standardwerte");
        return false;
    }
    _readCoeffs(EEPROM_ADDR_HEIGHT, height);
    _readCoeffs(EEPROM_ADDR_ROLL,   roll);
    _readCoeffs(EEPROM_ADDR_PITCH,  pitch);
    LOGGER_NOTICE("[EEPROM] Geladen");
    LOGGER_NOTICE_FMT("[EEPROM] height Kp=%.4f Ki=%.4f Kd=%.4f", height.kp, height.ki, height.kd);
    LOGGER_NOTICE_FMT("[EEPROM] roll   Kp=%.4f Ki=%.4f Kd=%.4f", roll.kp,   roll.ki,   roll.kd);
    LOGGER_NOTICE_FMT("[EEPROM] pitch  Kp=%.4f Ki=%.4f Kd=%.4f", pitch.kp,  pitch.ki,  pitch.kd);
    return true;
}

void Settings::reset() {
    EEPROM.write(EEPROM_VALID_ADDR, 0x00);
    EEPROM.commit();
    LOGGER_NOTICE("[EEPROM] Zurueckgesetzt");
}
