#pragma once

// Gemeinsames Interface fuer die main.cpp-Betriebsmodi (NORMALBETRIEB,
// TEST_KEYBOARD, kuenftige Testmodi). main.cpp waehlt zur Compile-Zeit genau
// eine Mode-Instanz aus (ueber "activeMode", analog zum "comm"-Pointer) und
// ruft danach nur noch deren setup()/loop() auf - ein neuer Modus bedeutet
// eine neue Mode-Unterklasse, main.cpp selbst bleibt unveraendert.
class Mode {
public:
    virtual ~Mode() = default;
    virtual void setup() = 0;
    virtual void loop() = 0;
};
