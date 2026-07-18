#pragma once

// Buendelt alle TEST_*-Codepfade aus main.cpp (siehe config.h fuer die
// Aktivierungsschalter). setup()/loop() werden unbedingt aus main.cpp
// aufgerufen; ist kein TEST_*-Schalter aktiv, kompilieren beide Methoden
// zu leeren Funktionen.
class TestModes {
public:
    static void setup();
    static void loop();
};
