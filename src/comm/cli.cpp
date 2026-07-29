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

    // ── Achsen fuer das "pid"-Kommando ─────────────────────────
    enum { AX_HEIGHT = 0, AX_ROLL, AX_PITCH, AX_COUNT };

    const char *axisName(int ax) {
        switch (ax) {
            case AX_HEIGHT: return "height";
            case AX_ROLL:   return "roll";
            default:        return "pitch";
        }
    }

    PIDController &axisPid(int ax) {
        switch (ax) {
            case AX_HEIGHT: return flightController.getPidHeight();
            case AX_ROLL:   return flightController.getPidRoll();
            default:        return flightController.getPidPitch();
        }
    }

    // Optionen case-insensitiv vergleichen: '-Kd', '-kd' und '-KD' sind
    // gleichwertig, wie auch die Kommandonamen der Shell selbst.
    bool optIs(const char *arg, const char *name) {
        return strcasecmp(arg, name) == 0;
    }

    // Ein Achsen-Objekt als JSON-Fragment, ohne umschliessende Klammern.
    void printAxisJson(int ax) {
        PIDController &p = axisPid(ax);
        shell.print('"');
        shell.print(axisName(ax));
        shell.print(F("\":{\"Kp\":"));  shell.print(p.getKp(), 4);
        shell.print(F(",\"Ki\":"));     shell.print(p.getKi(), 4);
        shell.print(F(",\"Kd\":"));     shell.print(p.getKd(), 4);
        shell.print('}');
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

    void pidHelp() {
        shell.println(F("pid - PID-Koeffizienten anzeigen und setzen"));
        shell.println(F("usage: pid [-height|-roll|-pitch] [-Kp <w>] [-Ki <w>] [-Kd <w>]"));
        shell.println(F("  pid                       alle Regler als JSON"));
        shell.println(F("  pid -height               nur den Hoehenregler als JSON"));
        shell.println(F("  pid -height -Kd 10        Kd des Hoehenreglers setzen"));
        shell.println(F("  pid -roll -Kp 1,2 -Ki 0.1 mehrere Koeffizienten auf einmal"));
        shell.println(F("  pid -height -Kp 2 -roll -Kp 1   mehrere Achsen je Aufruf"));
        shell.println(F("Die Achse gilt fuer alle folgenden -K-Optionen und muss vor"));
        shell.println(F("ihnen stehen. Optionen sind case-insensitiv, Komma als"));
        shell.println(F("Dezimaltrenner ist erlaubt. Ausgegeben werden die genannten"));
        shell.println(F("Achsen - ohne Achsenangabe alle."));
    }

    // Loest die frueheren neun setK*-Kommandos und getPid ab. Die Achse ist
    // zustandsbehaftet: sie gilt fuer alle nachfolgenden -K-Optionen, daher
    // laesst sich in einem Aufruf auch mehr als ein Regler stellen.
    int cmdPid(int argc, char **argv) {
        if (argc >= 2 && optIs(argv[1], "-h")) {
            pidHelp();
            return 0;
        }

        bool selected[AX_COUNT] = {false, false, false};
        bool anySelected = false;
        int  current     = -1;   // aktuelle Achse, -1 = noch keine gewaehlt

        for (int i = 1; i < argc; ++i) {
            const char *a = argv[i];

            int ax = -1;
            if      (optIs(a, "-height")) ax = AX_HEIGHT;
            else if (optIs(a, "-roll"))   ax = AX_ROLL;
            else if (optIs(a, "-pitch"))  ax = AX_PITCH;

            if (ax >= 0) {
                current = ax;
                selected[ax] = true;
                anySelected  = true;
                continue;
            }

            bool isKp = optIs(a, "-Kp");
            bool isKi = optIs(a, "-Ki");
            bool isKd = optIs(a, "-Kd");
            if (isKp || isKi || isKd) {
                // Bewusst keine Vorgabe-Achse: sonst landet ein vergessenes
                // -height still im Hoehenregler statt im gemeinten.
                if (current < 0) {
                    shell.println(F("pid: erst Achse waehlen (-height, -roll oder -pitch)"));
                    return -1;
                }
                if (i + 1 >= argc) {
                    shell.print(F("pid: Wert fehlt nach "));
                    shell.println(a);
                    return -1;
                }
                float v = parseFloatDe(argv[++i]);
                PIDController &p = axisPid(current);
                if      (isKp) p.setKp(v);
                else if (isKi) p.setKi(v);
                else           p.setKd(v);
                continue;
            }

            shell.print(F("pid: unbekannte Option "));
            shell.println(a);
            shell.println(F("pid -h fuer Hilfe"));
            return -1;
        }

        shell.print('{');
        bool first = true;
        for (int ax = 0; ax < AX_COUNT; ++ax) {
            if (anySelected && !selected[ax]) continue;
            if (!first) shell.print(',');
            printAxisJson(ax);
            first = false;
        }
        shell.println('}');
        return 0;
    }
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
        shell.addCommand(F("pid - PID anzeigen/setzen, 'pid -h' fuer Optionen"), cmdPid);

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
