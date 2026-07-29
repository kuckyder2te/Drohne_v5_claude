#include "myLogger.h"
#include "config.h"

// Schreibt direkt auf die Hardware-Streams; welcher Kanal Ausgaben bekommt,
// steuern _SERIAL_LOG (USB) und _BT_LOG (BT-UART) in config.h. Bewusst
// unabhaengig davon, an welchem Stream die CLI-Shell haengt - so laesst sich
// die Bedienung per CLI_USE_BLUETOOTH auf BT legen und trotzdem am
// USB-Kabel mitlesen. (Vorher lief alles ueber CommChannel, wodurch die
// beiden Flags faktisch wirkungslos waren.)
void dlog(const String& msg) {
#ifdef _SERIAL_LOG
    Serial.println(msg);
#endif
#ifdef _BT_LOG
    BT_UART.println(msg);
#endif
}
