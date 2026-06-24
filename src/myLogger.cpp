#include "myLogger.h"
#include "config.h"
#ifdef _BT_LOG
#include "comm/BluetoothComm.h"
extern BluetoothComm bt;
#endif

void dlog(const String& msg) {
#ifdef _SERIAL_LOG
    Serial.println(msg);
#endif
#ifdef _BT_LOG
    bt.sendLine(msg.c_str());
#endif
}
