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

    // Parst eine Zahl, Komma wie Punkt als Dezimaltrenner (deutsche Tastatur).
    //
    // Gibt false zurueck, wenn der String keine vollstaendig verwertbare Zahl
    // ist. Das ist der eigentliche Zweck: strtof allein liefert fuer "abc"
    // still 0.0f, ein vertipptes "pid -height -Kp o.5" wuerde den Beiwert
    // also unbemerkt auf null setzen statt sich zu beschweren.
    bool parseFloatDe(const char *s, float &out) {
        if (!s || !*s) return false;

        char buf[24];
        if (strlen(s) >= sizeof(buf)) return false;  // sonst wuerde gekuerzt
        strcpy(buf, s);
        for (char *p = buf; *p; ++p)
            if (*p == ',') *p = '.';

        char *end = nullptr;
        float v = strtof(buf, &end);

        if (end == buf) return false;            // keine Ziffer gefunden
        while (*end == ' ' || *end == '\t') ++end;
        if (*end != '\0') return false;          // Rest wie "12abc" oder "1.2.3"
        if (!isfinite(v)) return false;          // "nan", "inf", Ueberlauf

        out = v;
        return true;
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

    // Standardwerte je Achse aus config.h - Ziel von "pid -reset".
    PidCoeffs axisDefaults(int ax) {
        switch (ax) {
            case AX_HEIGHT: return {PID_KP_HEIGHT, PID_KI_HEIGHT, PID_KD_HEIGHT};
            case AX_ROLL:   return {PID_KP_ROLL,   PID_KI_ROLL,   PID_KD_ROLL};
            default:        return {PID_KP_PITCH,  PID_KI_PITCH,  PID_KD_PITCH};
        }
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

    // Gibt die markierten Achsen als JSON-Objekt aus; mask == nullptr -> alle.
    void printPidJson(const bool *mask) {
        shell.print('{');
        bool first = true;
        for (int ax = 0; ax < AX_COUNT; ++ax) {
            if (mask && !mask[ax]) continue;
            if (!first) shell.print(',');
            printAxisJson(ax);
            first = false;
        }
        shell.println('}');
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

    // Ueberschattet das eingebaute "help" der Bibliothek: addCommand() haengt
    // bei Namensgleichheit VOR den bestehenden Eintrag, und execute() nimmt
    // den ersten Treffer. So laesst sich die generierte Kommandoliste
    // weiterverwenden und um die Sofort-Tasten ergaenzen, die in ihr gar
    // nicht auftauchen koennen - sie sind keine Shell-Kommandos.
    int cmdHelp(int argc, char **argv) {
        SimpleSerialShell::printHelp(argc, argv);

        shell.println();
        shell.println(F("Sofort-Tasten (einzelne Taste, ohne Enter):"));
        shell.println(F("  d   DISARM - Motoren sofort auf Minimum ('D' ebenso)"));
        shell.println(F("  +   Zielhoehe um 10 cm erhoehen"));
        shell.println(F("  -   Zielhoehe um 10 cm verringern"));
        shell.println();
        shell.println(F("Diese drei Zeichen werden vor der Shell aus dem Datenstrom"));
        shell.println(F("gefischt und wirken daher schon beim Tastendruck, ohne Enter"));
        shell.println(F("und ohne Zeilenende. Gedacht ist das als Not-Aus, der im"));
        shell.println(F("Ernstfall nicht erst eine Zeile zu Ende getippt haben will."));
        shell.println(F("Mehrfach druecken addiert sich: '+++' sind +30 cm."));
        shell.println();
        shell.println(F("Sie gelten NUR am Zeilenanfang. Mitten in einer Eingabe sind"));
        shell.println(F("es gewoehnliche Zeichen - sonst wuerde 'setHeight -10' sein"));
        shell.println(F("Minus verlieren. Wurde eine Zeile angefangen und nicht"));
        shell.println(F("abgeschickt, verwirft die CLI sie nach 5 s ohne weitere"));
        shell.println(F("Eingabe, damit der Not-Aus wieder scharf ist."));
        shell.println(F("Aus demselben Grund faengt kein Kommandoname mit 'd' an;"));
        shell.println(F("DISARM heisst als Kommando deshalb 'stop'."));
        shell.println();
        shell.println(F("Optionen einzelner Kommandos: 'pid -h'"));
        return 0;
    }

    // ── Werte (setX/getX) ──────────────────────────────────────

    int cmdSetHeight(int argc, char **argv) {
        if (argc < 2) {
            shell.println(F("usage: setHeight <cm>"));
            return -1;
        }
        float cm;
        if (!parseFloatDe(argv[1], cm)) {
            shell.print(F("setHeight: keine gueltige Zahl: "));
            shell.println(argv[1]);
            return -1;
        }
        flightController.setTargetHeightCm(cm);
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
        shell.println(F("pid - PID-Koeffizienten anzeigen, setzen, sichern"));
        shell.println(F("usage: pid [-reset] [-height|-roll|-pitch] [-Kp|-Ki|-Kd <w>] [-save]"));
        shell.println(F("  pid                       alle Regler als JSON"));
        shell.println(F("  pid -height               nur den Hoehenregler als JSON"));
        shell.println(F("  pid -height -Kd 10        Kd des Hoehenreglers setzen"));
        shell.println(F("  pid -roll -Kp 1,2 -Ki 0.1 mehrere Koeffizienten auf einmal"));
        shell.println(F("  pid -height -Kp 2 -roll -Kp 1   mehrere Achsen je Aufruf"));
        shell.println(F("  pid -save                 alle drei Regler ins EEPROM"));
        shell.println(F("  pid -reset                alle drei auf Standardwerte"));
        shell.println(F("  pid -height -Kp 2 -save   aendern und sichern in einem Zug"));
        shell.println(F("Die Achse gilt fuer alle folgenden -K-Optionen und muss vor"));
        shell.println(F("ihnen stehen. Optionen sind case-insensitiv, Komma als"));
        shell.println(F("Dezimaltrenner ist erlaubt. Ausgegeben werden die genannten"));
        shell.println(F("Achsen - ohne Achsenangabe alle."));
        shell.println(F("Ein Wert, der keine gueltige Zahl ist, bricht den ganzen"));
        shell.println(F("Aufruf ab - es wird dann nichts gesetzt, sichert und"));
        shell.println(F("zurueckgesetzt auch nicht."));
        shell.println(F("-save/-reset gelten IMMER fuer alle drei Regler (das EEPROM"));
        shell.println(F("hat nur einen Gueltigkeitsmarker) und werden unabhaengig von"));
        shell.println(F("ihrer Position ausgefuehrt: erst -reset, dann die -K-Werte,"));
        shell.println(F("zuletzt -save."));
    }

    // Loest die frueheren neun setK*-Kommandos sowie getPid, save und reset
    // ab. Die Achse ist zustandsbehaftet: sie gilt fuer alle nachfolgenden
    // -K-Optionen, daher laesst sich in einem Aufruf mehr als ein Regler
    // stellen.
    int cmdPid(int argc, char **argv) {
        if (argc >= 2 && optIs(argv[1], "-h")) {
            pidHelp();
            return 0;
        }

        // Durchgang 1: -reset/-save nur einsammeln. Beide wirken auf alle
        // Achsen und werden in fester Reihenfolge ausgefuehrt (reset -> Werte
        // -> save), damit "pid -save -height -Kp 2" nicht den Stand VOR der
        // Aenderung sichert - die Position im Aufruf soll egal sein.
        bool doReset = false;
        bool doSave  = false;
        for (int i = 1; i < argc; ++i) {
            if      (optIs(argv[i], "-reset")) doReset = true;
            else if (optIs(argv[i], "-save"))  doSave  = true;
        }

        // Durchgang 2: alles pruefen und die Schreibvorgaenge nur SAMMELN.
        // Angewendet wird erst in Durchgang 3, wenn die ganze Zeile fehlerfrei
        // ist - sonst hinterliesse ein Tippfehler im dritten Argument die
        // ersten beiden bereits gesetzt, und man taete beim Tuning mit einem
        // halb geaenderten Regler weiter.
        struct PidWrite { int8_t axis; char which; float value; };
        PidWrite writes[8];
        uint8_t  nWrites = 0;

        bool selected[AX_COUNT] = {false, false, false};
        bool anySelected = false;
        int  current     = -1;   // aktuelle Achse, -1 = noch keine gewaehlt

        for (int i = 1; i < argc; ++i) {
            const char *a = argv[i];

            if (optIs(a, "-reset") || optIs(a, "-save")) continue;  // s.o.

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
                const char *valStr = argv[++i];
                float v;
                if (!parseFloatDe(valStr, v)) {
                    shell.print(F("pid: keine gueltige Zahl nach "));
                    shell.print(a);
                    shell.print(F(": "));
                    shell.println(valStr);
                    return -1;
                }
                if (nWrites >= (uint8_t)(sizeof(writes) / sizeof(writes[0]))) {
                    shell.println(F("pid: zu viele Werte in einem Aufruf"));
                    return -1;
                }
                writes[nWrites].axis  = (int8_t)current;
                writes[nWrites].which = isKp ? 'P' : (isKi ? 'I' : 'D');
                writes[nWrites].value = v;
                ++nWrites;
                continue;
            }

            shell.print(F("pid: unbekannte Option "));
            shell.println(a);
            shell.println(F("pid -h fuer Hilfe"));
            return -1;
        }

        // Durchgang 3: ab hier kann nichts mehr fehlschlagen
        if (doReset) {
            settings.reset();
            for (int ax = 0; ax < AX_COUNT; ++ax) {
                PidCoeffs d = axisDefaults(ax);
                PIDController &p = axisPid(ax);
                p.setKp(d.kp);
                p.setKi(d.ki);
                p.setKd(d.kd);
            }
        }

        for (uint8_t w = 0; w < nWrites; ++w) {
            PIDController &p = axisPid(writes[w].axis);
            switch (writes[w].which) {
                case 'P': p.setKp(writes[w].value); break;
                case 'I': p.setKi(writes[w].value); break;
                default:  p.setKd(writes[w].value); break;
            }
        }

        if (doSave) {
            PidCoeffs c[AX_COUNT];
            for (int ax = 0; ax < AX_COUNT; ++ax) {
                PIDController &p = axisPid(ax);
                c[ax] = {p.getKp(), p.getKi(), p.getKd()};
            }
            settings.save(c[AX_HEIGHT], c[AX_ROLL], c[AX_PITCH]);
        }

        // Nach -save/-reset immer alle Achsen zeigen: beide betreffen alle
        // drei, eine auf eine Achse eingeschraenkte Ausgabe waere irrefuehrend.
        printPidJson((anySelected && !doSave && !doReset) ? selected : nullptr);
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
        // Muss vor der eingebauten Liste stehen -> siehe cmdHelp()
        shell.addCommand(F("help - Kommandos und Sofort-Tasten"), cmdHelp);

        shell.addCommand(F("arm - ARM (2x innerhalb 3 s bestaetigen)"), cmdArm);
        shell.addCommand(F("stop - DISARM, Motoren sofort stoppen"), cmdStop);
        shell.addCommand(F("recalibrate - Barometer rekalibrieren (nur disarmt)"), cmdRecalibrate);
        shell.addCommand(F("statusLog - Statusausgabe ein/aus"), cmdStatusLog);

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
