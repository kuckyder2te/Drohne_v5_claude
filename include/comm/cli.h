#pragma once

#include <Arduino.h>

// Duennes Wrapper-Modul um philj404/SimpleSerialShell - die alleinige
// Bedienoberflaeche der Firmware (loest den frueheren CommChannel samt
// InputHandler/KeyEvent ab). Benennung: Aktionen als Verben (arm, stop,
// recalibrate, save, reset, statusLog), Werte als setX/getX (setHeight,
// setKpRoll, getPid, ...).
namespace cli {
    // Registriert alle CLI-Befehle und bindet die Shell an `stream`.
    // Die Shell ist ein Singleton mit genau EINEM Stream (Serial oder
    // Serial1, Auswahl per CLI_USE_BLUETOOTH in config.h), daher genuegt
    // ein einmaliges attach() fuer die gesamte Laufzeit.
    void begin(Stream &stream);

    // Einmal pro loop()-Durchlauf aufrufen.
    // Rueckgabe true, wenn in diesem Durchlauf Bytes verarbeitet wurden
    // (Not-Aus-Taste, fertige Kommandozeile oder Teilzeile im Puffer); der
    // Rueckgabewert ist rein informativ, NormalMode wertet ihn nicht aus.
    //
    // Fischt 'd' (Not-Aus), '+' und '-' vor der Shell aus dem Stream, damit
    // sie ohne Enter wirken - aber nur am Zeilenanfang, sonst verschwaende
    // das '-' in "setHeight -10". Deshalb darf auch kein Kommandoname mit
    // 'd' beginnen (siehe cli.cpp).
    bool update();
}
