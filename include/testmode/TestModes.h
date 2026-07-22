#pragma once

// Buendelt den TEST_KEYBOARD-Codepfad aus main.cpp (siehe config.h fuer den
// Aktivierungsschalter). Die anderen TEST_*-Modi sind eigenstaendige
// PlatformIO-Umgebungen unter test/ (siehe platformio.ini). setup()/loop()
// werden unbedingt aus main.cpp aufgerufen; ist TEST_KEYBOARD nicht aktiv,
// kompilieren beide Methoden zu leeren Funktionen.
class TestModes {
public:
    static void setup();
    static void loop();
};
