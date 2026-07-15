#include "myLogger.h"
#include "config.h"
#include "comm/CommChannel.h"
extern CommChannel* comm;

void dlog(const String& msg) {
#if defined(_SERIAL_LOG) || defined(_BT_LOG)
    comm->sendLine(msg.c_str());
#endif
}
