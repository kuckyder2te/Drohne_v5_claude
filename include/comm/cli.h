#pragma once

#include <Arduino.h>

// Duennes Wrapper-Modul um philj404/SimpleSerialShell. Koexistiert vorerst
// mit comm/CommChannel.h (ersetzt es noch nicht) - erster Schritt der
// schrittweisen Umstellung der Bedienung auf eine reine CLI.
namespace cli {
    // Registriert alle CLI-Befehle und bindet die Shell an `stream`.
    // `stream` ist derselbe Stream, an den auch der aktive CommChannel
    // gebunden ist (Serial oder Serial1, Auswahl per COMM_USE_BLUETOOTH in
    // config.h) - es ist nie mehr als ein physischer Stream gleichzeitig
    // aktiv, daher genuegt ein einmaliges attach() fuer die gesamte Laufzeit.
    void begin(Stream &stream);

    // Einmal pro loop()-Durchlauf aufrufen, VOR comm->getKey().
    // Rueckgabe true: dieser Durchlauf gehoert der CLI (Zeile beginnt mit
    // ':' oder eine solche Zeile wird gerade eingelesen) - der Aufrufer
    // muss comm->getKey()/getCommand()/processCommand() fuer diesen
    // Durchlauf auslassen, da sonst beide Parser um dieselben Bytes
    // konkurrieren wuerden.
    // Rueckgabe false: nichts CLI-relevantes anstehend, der bestehende
    // CommChannel-Pfad darf normal weiterlaufen.
    bool update();
}
