#pragma once

#include "control/Mode.h"

// Betriebsmodus fuer echten Flugbetrieb (config.h: NORMALBETRIEB). Sensor-Init,
// Sicherheits-Check, Input-Handling und der FlightController-Regelkreis -
// vorher direkt per #ifdef NORMALBETRIEB in main.cpp::setup()/loop().
class NormalMode : public Mode {
public:
    void setup() override;
    void loop() override;
};
