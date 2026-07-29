#include "comm/cli.h"
#include "config.h"
#include "myLogger.h"
#include "control/FlightController.h"
#include "storage/Settings.h"
#include "Barometer.h"
#include "IMU.h"
#include <SimpleSerialShell.h>

// Reine C-Funktionszeiger (SimpleSerialShell::CommandFunction) koennen keinen
// Zustand einfangen - Zugriff auf die in src/mode/NormalMode.cpp definierten
// Globalen daher ueber extern.
extern FlightController flightController;
extern Settings         settings;
extern Barometer        baro;
extern IMU              imu;

namespace {
    Stream *cliStream = nullptr;

    // Not-Aus-Tasten ('d', '+', '-') werden in update() aus dem Stream gefischt,
    // BEVOR die Shell sie sieht - aber nur am Zeilenanfang, sonst verschwaende
    // das '-' in "setHeight -10" als Sofortbefehl.
    bool     atLineStart = true;
    uint32_t lastByteMs  = 0;
    constexpr uint32_t IDLE_RESET_MS = 5000;

    // Quittungen laufen ueber shell.print*(), damit sie immer auf dem Kanal
    // landen, von dem der Befehl kam. LOG() geht daneben unabhaengig davon
    // an Serial und/oder BT_UART (Flags _SERIAL_LOG/_BT_LOG in config.h).

    // Komma als Dezimaltrennzeichen akzeptieren (deutsche Tastatur).
    float parseFloatDe(const char *s) {
        char buf[16];
        strncpy(buf, s, sizeof(buf) - 1);
        buf[sizeof(buf) - 1] = '\0';
        for (char *p = buf; *p; ++p)
            if (*p == ',') *p = '.';
        return strtof(buf, nullptr);
    }

    // Gemeinsamer Rumpf aller neun Koeffizienten-Setter; 'which' waehlt
    // Kp/Ki/Kd. Die Kommandos selbst bleiben dadurch Einzeiler.
    int setCoeff(int argc, char **argv, PIDController &pid, char which, const char *label) {
        if (argc < 2) {
            shell.print(F("usage: "));
            shell.print(argv[0]);
            shell.println(F(" <wert>"));
            return -1;
        }
        float v = parseFloatDe(argv[1]);
        switch (which) {
            case 'P': pid.setKp(v); break;
            case 'I': pid.setKi(v); break;
            case 'D': pid.setKd(v); break;
        }
        shell.print(label);
        shell.print('=');
        shell.println(v, 4);
        return 0;
    }

    void printCoeffs(const char *label, PIDController &pid) {
        shell.print(label);
        shell.print(F(" Kp=")); shell.print(pid.getKp(), 4);
        shell.print(F(" Ki=")); shell.print(pid.getKi(), 4);
        shell.print(F(" Kd=")); shell.println(pid.getKd(), 4);
    }

    void printTargetHeight() {
        shell.print(F("targetHeightCm="));
        shell.println(flightController.getTargetHeightCm());
    }

    // ── Aktionen (Verben) ──────────────────────────────────────

    int cmdArm(int /*argc*/, char ** /*argv*/) {
        bool wasArmed = flightController.isArmed();
        flightController.requestArm(imu.isReady(), baro);

        if (flightController.isArmed())
            shell.println(wasArmed ? F("[CLI] bereits ARMED")
                                   : F("[CLI] ARMED - Ziel 20 cm"));
        else if (!imu.isReady())
            shell.println(F("[CLI] ARM verweigert - IMU nicht bereit"));
        else
            shell.println(F("[CLI] ARM? 'arm' nochmal innerhalb 3 s"));
        return 0;
    }

    int cmdStop(int /*argc*/, char ** /*argv*/) {
        flightController.disarm();
        shell.println(F("[CLI] DISARM - Motoren gestoppt"));
        return 0;
    }

    int cmdRecalibrate(int /*argc*/, char ** /*argv*/) {
        if (flightController.isArmed()) {
            shell.println(F("[CLI] Rekalibrierung nur im DISARM Modus"));
            return -1;
        }
        flightController.recalibrate(baro);
        shell.println(F("[CLI] Barometer rekalibriert"));
        return 0;
    }

    int cmdStatusLog(int /*argc*/, char ** /*argv*/) {
        flightController.toggleStatusLog();
        // Der Statuslog selbst laeuft per LOG() nach BT, nicht hierher.
        shell.println(F("[CLI] Statuslog umgeschaltet (Ausgabe laeuft ueber BT)"));
        return 0;
    }

    int cmdSave(int /*argc*/, char ** /*argv*/) {
        PIDController &h = flightController.getPidHeight();
        settings.save(h.getKp(), h.getKi(), h.getKd());
        shell.println(F("[CLI] Hoehen-PID gespeichert"));
        return 0;
    }

    int cmdReset(int /*argc*/, char ** /*argv*/) {
        settings.reset();
        PIDController &h = flightController.getPidHeight();
        h.setKp(PID_KP_HEIGHT);
        h.setKi(PID_KI_HEIGHT);
        h.setKd(PID_KD_HEIGHT);
        shell.println(F("[CLI] Hoehen-PID auf Standard zurueckgesetzt"));
        return 0;
    }

    // ── Werte (setX/getX) ──────────────────────────────────────

    int cmdSetHeight(int argc, char **argv) {
        if (argc < 2) {
            shell.println(F("usage: setHeight <cm>"));
            return -1;
        }
        flightController.setTargetHeightCm(parseFloatDe(argv[1]));
        printTargetHeight();
        return 0;
    }

    int cmdGetHeight(int /*argc*/, char ** /*argv*/) {
        printTargetHeight();
        return 0;
    }

    int cmdGetArmed(int /*argc*/, char ** /*argv*/) {
        shell.print(F("armed="));
        shell.println(flightController.isArmed() ? F("1") : F("0"));
        return 0;
    }

    int cmdGetPid(int /*argc*/, char ** /*argv*/) {
        printCoeffs("Hoehe", flightController.getPidHeight());
        printCoeffs("Roll ", flightController.getPidRoll());
        printCoeffs("Pitch", flightController.getPidPitch());
        return 0;
    }

    int cmdSetKpHeight(int c, char **v) { return setCoeff(c, v, flightController.getPidHeight(), 'P', "Hoehe Kp"); }
    int cmdSetKiHeight(int c, char **v) { return setCoeff(c, v, flightController.getPidHeight(), 'I', "Hoehe Ki"); }
    int cmdSetKdHeight(int c, char **v) { return setCoeff(c, v, flightController.getPidHeight(), 'D', "Hoehe Kd"); }
    int cmdSetKpRoll  (int c, char **v) { return setCoeff(c, v, flightController.getPidRoll(),   'P', "Roll Kp");  }
    int cmdSetKiRoll  (int c, char **v) { return setCoeff(c, v, flightController.getPidRoll(),   'I', "Roll Ki");  }
    int cmdSetKdRoll  (int c, char **v) { return setCoeff(c, v, flightController.getPidRoll(),   'D', "Roll Kd");  }
    int cmdSetKpPitch (int c, char **v) { return setCoeff(c, v, flightController.getPidPitch(),  'P', "Pitch Kp"); }
    int cmdSetKiPitch (int c, char **v) { return setCoeff(c, v, flightController.getPidPitch(),  'I', "Pitch Ki"); }
    int cmdSetKdPitch (int c, char **v) { return setCoeff(c, v, flightController.getPidPitch(),  'D', "Pitch Kd"); }
}

namespace cli {
    void begin(Stream &stream) {
        cliStream = &stream;

        // Benennung: Aktionen als Verben, Werte als setX/getX (analog
        // PIDController::setKp()/getKp()).
        //
        // WICHTIG: Kein Kommandoname darf mit 'd' beginnen! 'd' wird in
        // update() byte-sofort als Not-Aus abgefangen, bevor die Shell es
        // sieht - ein Befehl "disarm" wuerde daher schon beim ersten Byte
        // ausloesen und "isarm" im Puffer hinterlassen. Deshalb "stop".
        shell.addCommand(F("arm - ARM (2x innerhalb 3 s bestaetigen)"), cmdArm);
        shell.addCommand(F("stop - DISARM, Motoren sofort stoppen"), cmdStop);
        shell.addCommand(F("recalibrate - Barometer rekalibrieren (nur disarmt)"), cmdRecalibrate);
        shell.addCommand(F("statusLog - Statusausgabe ein/aus"), cmdStatusLog);
        shell.addCommand(F("save - Hoehen-PID im EEPROM speichern"), cmdSave);
        shell.addCommand(F("reset - Hoehen-PID auf Standardwerte"), cmdReset);

        shell.addCommand(F("setHeight cm - Zielhoehe setzen"), cmdSetHeight);
        shell.addCommand(F("getHeight - Zielhoehe ausgeben"), cmdGetHeight);
        shell.addCommand(F("getArmed - Flugzustand ausgeben"), cmdGetArmed);
        shell.addCommand(F("getPid - alle PID-Koeffizienten ausgeben"), cmdGetPid);

        shell.addCommand(F("setKpHeight wert - Hoehe Kp"), cmdSetKpHeight);
        shell.addCommand(F("setKiHeight wert - Hoehe Ki"), cmdSetKiHeight);
        shell.addCommand(F("setKdHeight wert - Hoehe Kd"), cmdSetKdHeight);
        shell.addCommand(F("setKpRoll wert - Roll Kp"), cmdSetKpRoll);
        shell.addCommand(F("setKiRoll wert - Roll Ki"), cmdSetKiRoll);
        shell.addCommand(F("setKdRoll wert - Roll Kd"), cmdSetKdRoll);
        shell.addCommand(F("setKpPitch wert - Pitch Kp"), cmdSetKpPitch);
        shell.addCommand(F("setKiPitch wert - Pitch Ki"), cmdSetKiPitch);
        shell.addCommand(F("setKdPitch wert - Pitch Kd"), cmdSetKdPitch);

        shell.attach(stream);

        // Lebenszeichen auf dem Shell-Kanal: LOG() geht je nach
        // _SERIAL_LOG/_BT_LOG ggf. woandershin, dann waere hier sonst nichts
        // zu sehen und der Kanal wirkte tot.
        shell.println();
        shell.println(F("[CLI] bereit - 'help' listet alle Befehle"));
        shell.println(F("[CLI] sofort ohne Enter: d = DISARM, +/- = Zielhoehe +/-10 cm"));
    }

    bool update() {
        if (!cliStream) return false;

        bool consumed = false;

        // Not-Aus-Tasten vor der Shell abfangen - nur am Zeilenanfang, denn
        // mitten in einer Zeile gehoert '-' zum Argument ("setHeight -10").
        while (atLineStart && cliStream->available()) {
            int c = cliStream->peek();
            if (c == 'd' || c == 'D') {
                cliStream->read();
                flightController.disarm();
                shell.println(F("[CLI] DISARM - Motoren gestoppt"));
            } else if (c == '+') {
                cliStream->read();
                flightController.adjustTargetHeight(10.0f);
                printTargetHeight();
            } else if (c == '-') {
                cliStream->read();
                flightController.adjustTargetHeight(-10.0f);
                printTargetHeight();
            } else {
                break;
            }
            lastByteMs = millis();
            consumed   = true;
        }

        bool hadInput = cliStream->available() > 0;
        if (hadInput) lastByteMs = millis();

        if (shell.executeIfInput()) {
            atLineStart = true;   // Zeile abgearbeitet -> Not-Aus wieder scharf
            consumed    = true;
        } else if (hadInput) {
            atLineStart = false;  // Teilzeile liegt im Shell-Puffer
            consumed    = true;
        } else if (!atLineStart && (millis() - lastByteMs) > IDLE_RESET_MS) {
            // Eine abgebrochene Eingabe darf den Not-Aus nicht dauerhaft
            // blockieren - Puffer verwerfen und wieder scharf schalten.
            shell.resetBuffer();
            shell.println();
            shell.println(F("[CLI] Eingabe verworfen (Timeout)"));
            atLineStart = true;
        }

        return consumed;
    }
}
