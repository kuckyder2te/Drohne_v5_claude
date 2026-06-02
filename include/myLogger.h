#pragma once

#pragma once

#include <Arduino.h>
#include "config.h"

// Vorwärts-Deklaration
void dlog(const String& msg);

// Komfort-Makros
#define LOG(msg);dlog(String(msg))
#define LOG_FMT(fmt, ...) { char _buf[100]; snprintf(_buf, sizeof(_buf), fmt, __VA_ARGS__); dlog(String(_buf)); }