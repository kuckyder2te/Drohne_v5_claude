#pragma once

#include <Arduino.h>
// WICHTIG: <Logger.h> muss VOR den LOGGER_*-Makros stehen. Die Klasse
// deklariert selbst Methoden namens log(); waere LOGGER_NOTICE hier schon
// als Makro bekannt und der Header wuerde erst danach eingebunden, expandierte
// das Makro mitten in die Klassendeklaration ("macro passed 3 arguments, but
// takes just 1"). Deshalb nie <Logger.h> direkt einbinden - immer nur diesen
// Header.
#include <Logger.h>
#include <stdio.h>

#if defined LOCAL_DEBUG || defined GLOBAL_DEBUG
//#if (defined LOCAL_DEBUG || defined GLOBAL_DEBUG) && defined DEBUG
// Gemeinsamer Formatierpuffer aller *_FMT-Makros.
// ACHTUNG: die Makros nutzen das unbegrenzte sprintf(). Die laengste Zeile im
// Projekt ist die Statusausgabe in FlightController::logStatus() mit rund 122
// Zeichen - daher 160 mit Reserve. Wer eine laengere Formatzeichenkette
// hinzufuegt, muss den Puffer mitwachsen lassen. Groesse muss in JEDER
// Definition identisch sein (src/myLogger.cpp und je einmal pro Tool unter
// src/tools/), sonst ODR-Verstoss.
extern char logBuf[160];
    #define LOGGER_VERBOSE_FMT(fmt,...) sprintf(logBuf,fmt, __VA_ARGS__);LOGGER_VERBOSE(logBuf)
    #define LOGGER_NOTICE(msg) Logger::notice(__PRETTY_FUNCTION__, msg)
    #define LOGGER_NOTICE_CHK(chk1,chk2,msg) if(chk1!=chk2){chk2 = chk1;Logger::notice(__PRETTY_FUNCTION__, msg);}
    #define LOGGER_NOTICE_FMT(fmt,...) sprintf(logBuf,fmt, __VA_ARGS__);LOGGER_NOTICE(logBuf)
    #define LOGGER_NOTICE_FMT_CHK(chk1,chk2,fmt,...) if(chk1!=chk2){chk2 = chk1;sprintf(logBuf,fmt, __VA_ARGS__);LOGGER_NOTICE(logBuf);}      
    #define LOGGER_WARNING_FMT(fmt,...) sprintf(logBuf,fmt, __VA_ARGS__);LOGGER_WARNING(logBuf)
    #define LOGGER_ERROR_FMT(fmt,...) sprintf(logBuf,fmt, __VA_ARGS__);LOGGER_ERROR(logBuf)
    #define LOGGER_FATAL_FMT(fmt,...) sprintf(logBuf,fmt, __VA_ARGS__);LOGGER_FATAL(logBuf)
    #define LOGGER_SILENT_FMT(fmt,...) sprintf(logBuf,fmt, __VA_ARGS__);LOGGER_SILENT(logBuf)
    #define LOGGER_VERBOSE(msg) Logger::verbose(__PRETTY_FUNCTION__, msg)      
    #define LOGGER_WARNING(msg) Logger::warning(__PRETTY_FUNCTION__, msg)
    #define LOGGER_ERROR(msg) Logger::error(__PRETTY_FUNCTION__, msg)
    #define LOGGER_FATAL(msg) Logger::fatal(__PRETTY_FUNCTION__, msg)
    #define LOGGER_SILENT(msg) Logger::silent(__PRETTY_FUNCTION__, msg)
#else
    #define LOGGER_VERBOSE_FMT(...) asm volatile ("nop\n\t")
    #define LOGGER_NOTICE(...) asm volatile ("nop\n\t")
    #define LOGGER_NOTICE_CHK(...) asm volatile ("nop\n\t")    
    #define LOGGER_NOTICE_FMT(...) asm volatile ("nop\n\t")
    #define LOGGER_NOTICE_FMT_CHK(...) asm volatile ("nop\n\t")
    #define LOGGER_WARNING_FMT(...) asm volatile ("nop\n\t")
    #define LOGGER_ERROR_FMT(...) asm volatile ("nop\n\t")
    #define LOGGER_FATAL_FMT(...) asm volatile ("nop\n\t")
    #define LOGGER_SILENT_FMT(...) asm volatile ("nop\n\t")
    #define LOGGER_VERBOSE(...) asm volatile ("nop\n\t")
    #define LOGGER_WARNING(...) asm volatile ("nop\n\t")
    #define LOGGER_ERROR(...) asm volatile ("nop\n\t")
    #define LOGGER_FATAL(...) asm volatile ("nop\n\t")
    // #define LOGGER_FATAL(msg) Logger::fatal(__PRETTY_FUNCTION__, msg)   // should be visible even LOCAL_DEBUG is undefined
    #define LOGGER_SILENT(...) asm volatile ("nop\n\t")
#endif
void localLogger(Logger::Level level, const char* module, const char* message);
#undef LOCAL_DEBUG
