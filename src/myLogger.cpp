#include "myLogger.h"
#include "config.h"

void dlog(const String& msg) {
#ifdef _SERIAL_LOG
    Serial.println(msg);
#endif
#ifdef _BT_LOG
    BT_UART.println(msg);
#endif
}
