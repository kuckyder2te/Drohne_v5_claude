#include "myLogger.h"
#include "config.h"
#include "comm/CommChannel.h"
extern CommChannel* comm;

void dlog(const String& msg) {
#ifdef _SERIAL_LOG
    comm->sendLine(msg.c_str());
#endif
#ifdef _BT_LOG
    comm->sendLine(msg.c_str());
#endif
}
