#include "myLogger.h"
#include "comm/BluetoothConfig.h"

extern HardwareSerial* DebugSerial;

void dlog(const String& msg) {
#ifdef _SERIAL_LOG
    Serial.println(msg);
#endif
#ifdef _BT_LOG
    Serial1.println(msg);  // BT_UART = Serial1
#endif
}