#pragma once

// Alleiniger Firmware-Einstiegspunkt: kapselt Komposition und Ablauf des
// echten Flugbetriebs. NormalMode besitzt (als freie Globale in NormalMode.cpp)
// die Sensor-/Comm-Objekte und den FlightController, initialisiert in setup()
// Kommunikationskanal (comm/cli) und Sensorik und faehrt in loop() den
// Regelkreis. main.cpp inkludiert nur diesen Header und ruft setup()/loop().
//
// Die Hardware-Testwerkzeuge (inkl. Tastatur-/CLI-Test) sind eigenstaendige
// Programme unter test/ und beruehren diese Firmware nicht (siehe test/README).
class NormalMode {
public:
    static void setup();
    static void loop();
};
