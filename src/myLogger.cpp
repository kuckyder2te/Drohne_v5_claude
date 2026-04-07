#include "myLogger.h"

char logBuf[100];

void localLogger(Logger::Level level, const char* module, const char* message) {
    // ── Nachricht zusammenbauen ────────────────────────────
    // Format: [LEVEL]:module:message

    #ifdef LOG_TIMESTAMP
        String ts = String(millis()) + " - ";
    #else
        String ts = "";
    #endif

    String line = ts + "[" + Logger::asString(level) + "]:";
    if (strlen(module) > 0) {
        line += module;
        line += ":";
    }
    line += message;

    // ── Ausgabe auf Serial (USB) ───────────────────────────
    #ifdef _SERIAL_LOG
        Serial.println(line);
    #endif

    // ── Ausgabe auf Bluetooth ──────────────────────────────
    #ifdef _BT_LOG
        if (BT_UART) {  // nur wenn BT verbunden
            BT_UART.println(line);
        }
    #endif
}