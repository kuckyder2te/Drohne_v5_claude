#include "myLogger.h"
#include "config.h"

// Formatierpuffer der *_FMT-Makros (Groesse siehe Kommentar in myLogger.h).
// Die Tool-Umgebungen unter src/tools/ kompilieren diese Datei NICHT mit und
// bringen deshalb je eine eigene Definition mit.
char logBuf[160];

// Ausgabefunktion fuer bakercp/Logger, angemeldet per
// Logger::setOutputFunction() in NormalMode::setup().
//
// Schreibt direkt auf die Hardware-Streams; welcher Kanal Ausgaben bekommt,
// steuern _SERIAL_LOG (USB) und _BT_LOG (BT-UART) in config.h, beide
// gleichzeitig moeglich. Bewusst unabhaengig davon, an welchem Stream die
// CLI-Shell haengt - so laesst sich die Bedienung per CLI_USE_BLUETOOTH auf BT
// legen und trotzdem am USB-Kabel mitlesen.
void localLogger(Logger::Level level, const char *module, const char *message)
{
#if defined(_SERIAL_LOG) || defined(_BT_LOG)
    // Muss "[LEVEL]:" + Modul + ":" + logBuf fassen. Das Modul ist
    // __PRETTY_FUNCTION__ und wird lang: die Statuszeile aus
    // FlightController::logStatus() misst zusammengesetzt bereits 202 Zeichen.
    // 320 deckt den laengsten Modulnamen plus einen vollen logBuf[160] ab.
    // snprintf kuerzt notfalls, ueberschreibt aber nie.
    char line[320];
    int n = 0;

#ifdef LOG_TIMESTAMP
    n += snprintf(line + n, sizeof(line) - n, "%lu - ", (unsigned long)millis());
#endif
    n += snprintf(line + n, sizeof(line) - n, "[%s]:", Logger::asString(level));
    if (module && strlen(module) > 0)
        n += snprintf(line + n, sizeof(line) - n, "%s:", module);
    snprintf(line + n, sizeof(line) - n, "%s", message);
#endif

#ifdef _SERIAL_LOG
    Serial.println(line);
#endif
#ifdef _BT_LOG
    BT_UART.println(line);
#endif
}
