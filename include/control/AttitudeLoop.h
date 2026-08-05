#pragma once

#include <Arduino.h>

class IMU;
class MotorMixer;

// ── Lageregelkreis auf Kern 1 ──────────────────────────────────────────
//
// Laeuft mit ATTITUDE_RATE_HZ und besitzt waehrenddessen exklusiv den
// I2C-Bus und die Motor-PWM. Kern 0 fasst weder IMU noch MotorMixer an -
// ausser ueber den Not-Aus (MotorMixer::stopFast()), der reine
// Registerschreibzugriffe macht und deshalb von beiden Kernen sicher ist.
//
// Zwei Regeln gelten hier ausnahmslos:
//   1. Kein LOGGER_* auf Kern 1. Die *_FMT-Makros schreiben in den EINEN
//      globalen logBuf aus src/myLogger.cpp; parallele Ausgaben von Kern 0
//      wuerden sich gegenseitig zerstoeren. Meldungen laufen ueber
//      CoreTlm::abortCode, den Kern 0 in Klartext uebersetzt.
//   2. Nie auf Kern 0 warten. Alle Uebergaben sind so gebaut, dass Kern 1
//      im Zweifel mit der letzten bekannten Fassung weiterarbeitet.
namespace attitude {

void begin(IMU &imu, MotorMixer &motors);
void step();   // genau ein Regelzyklus, wird aus loop1() gerufen

} // namespace attitude
