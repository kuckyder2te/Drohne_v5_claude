#pragma once

#include <Arduino.h>

// Duennes Wrapper-Modul um philj404/SimpleSerialShell. Bietet inzwischen alle
// Befehle von comm/CommChannel.h an, koexistiert aber weiterhin mit ihm
// (ersetzt es noch nicht) - schrittweise Umstellung der Bedienung auf eine
// reine CLI. Benennung: Aktionen als Verben (arm, stop, recalibrate, save,
// reset, statusLog), Werte als setX/getX (setHeight, setKpRoll, getPid, ...).
namespace cli {
    // Registriert alle CLI-Befehle und bindet die Shell an `stream`.
    // `stream` ist derselbe Stream, an den auch der aktive CommChannel
    // gebunden ist (Serial oder Serial1, Auswahl per COMM_USE_BLUETOOTH in
    // config.h) - es ist nie mehr als ein physischer Stream gleichzeitig
    // aktiv, daher genuegt ein einmaliges attach() fuer die gesamte Laufzeit.
    void begin(Stream &stream);

    // Einmal pro loop()-Durchlauf aufrufen, VOR comm->getKey().
    // Rueckgabe true, wenn in diesem Durchlauf Bytes verarbeitet wurden
    // (Not-Aus-Taste, fertige Kommandozeile oder Teilzeile im Puffer).
    //
    // Solange COMM_USE_BLUETOOTH gesetzt ist, liegt die CLI auf USB und comm
    // auf BT - beide Parser konkurrieren dann nicht um dieselben Bytes, der
    // Rueckgabewert muss vom Aufrufer also nicht ausgewertet werden.
    //
    // Fischt 'd' (Not-Aus), '+' und '-' vor der Shell aus dem Stream, damit
    // sie ohne Enter wirken - aber nur am Zeilenanfang, sonst verschwaende
    // das '-' in "setHeight -10". Deshalb darf auch kein Kommandoname mit
    // 'd' beginnen (siehe cli.cpp).
    bool update();
}
