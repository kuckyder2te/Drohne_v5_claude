#pragma once

// Buendelt die Tastatur-/Kommando-Auswertung des Normalbetriebs (ARM-Timeout,
// switch(key)) aus main.cpp::loop(), damit dort im Wesentlichen nur noch
// Sensor-Update, Sicherheits-Check und der reine PID-Regelkreis stehen.
namespace InputHandler {
    void handle();
}
