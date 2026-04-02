/*
Autor: Stephan Scholz
Date : 2022:04:30

Erweiterung für LOGGER
Man kann jetzt einen Formatstring übergeben

LOGGER_NOTICE_FMT("/%f,%f,%f,%f,%f,%f,%f/",
                  a.acceleration.x,a.acceleration.y,a.acceleration.z,
                  g.gyro.x,g.gyro.y,g.gyro.z,temp.temperature);
daraus wird
[NOTICE]:virtual void Gyro::update():/-0.593762,0.090980,10.096349,0.029311,0.099657,-0.041568,21.868235/  
https://www.tutorialspoint.com/c_standard_library/c_function_sprintf.htm

Die Gesamtlänge der Nachricht darf 100 Bytes nicht überschreiten

Erweitert : 2022:07:08
Autor: Wilhelm Kuckelsberg
       via "#ifdef _MINITERM" kann nun die Ausgabe zwischen Serial und Serial2 gewechselt werden
       MINITERM ist das interne Terminal von PlatformIO

*/

#include "myLogger.h"

extern HardwareSerial *DebugOutput;
char logBuf[100];

void localLogger(Logger::Level level, const char* module, const char* message)
{
  #ifdef _MINITERM    // Ausgabe via USB

    #ifdef LOG_TIMESTAMP
      DebugOutput->print(millis());
      DebugOutput->print(" - ");
    #endif
      DebugOutput->print(F("["));
      DebugOutput->print(Logger::asString(level));
      DebugOutput->print(F("]:"));
      if (strlen(module) > 0)
      {
          DebugOutput->print(module);
          DebugOutput->print(":");
      }
      DebugOutput->println(message);
  #else
    #ifdef LOG_TIMESTAMP
      DebugOutput->print(millis());
      DebugOutput->print(" - ");
    #endif
      DebugOutput->print(F("["));
      DebugOutput->print(Logger::asString(level));
      DebugOutput->print(F("]:"));
      if (strlen(module) > 0)
      {
          DebugOutput->print(module);
          DebugOutput->print(":");
      }
      DebugOutput->println(message);
  #endif
}
