#pragma once

#include "control/Mode.h"

// Betriebsmodus TEST_KEYBOARD (config.h): BT/Keyboard-Befehlsecho + Barometer-
// Ausgabe, exercised gegen echte PIDController/Settings-Instanzen (ueber
// FlightController). Bleibt als in-firmware Mode statt eigenstaendiges
// test/-Tool, weil er den kompletten Input-/Tuning-Stack braucht, nicht nur
// einen einzelnen Hardware-Treiber.
class KeyboardTestMode : public Mode {
public:
    void setup() override;
    void loop() override;
};
