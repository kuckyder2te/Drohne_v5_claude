#!/usr/bin/env python3
"""Treibt die serielle CLI der Drohnen-Firmware (SimpleSerialShell auf USB/COM11)
und testet das Clamping von setHeight/getHeight.

Aufruf (PlatformIO-Python mit pyserial):
  ~/.platformio/penv/Scripts/python.exe height_test.py [--port COM11] [--baud 115200] [HOEHE ...]

Ohne HOEHE-Argumente wird der Standard-Sweep -10 .. 150 gefahren.

WICHTIG:
  * Nur USB-CLI (setHeight/getHeight/help). Es wird NICHT gearmt -> keine
    Motorbewegung. Armen geht ohnehin nur ueber den BT-Kanal (Taste 'a').
  * Die Shell terminiert Kommandos mit '\\r' (nicht '\\n') und hat einen '> '-Prompt.
  * LOG-Ausgaben der Firmware laufen ueber BT (Serial1), nicht ueber USB - auf
    COM11 erscheint nur die CLI-Antwort.
"""
import argparse
import sys
import time

try:
    import serial
except ImportError:
    print("FEHLER: pyserial fehlt. Mit PlatformIO-Python starten "
          "(~/.platformio/penv/Scripts/python.exe).")
    sys.exit(2)

DEFAULT_HEIGHTS = [-10, -5, 0, 4, 5, 6, 10, 33.3, 50, 99, 100, 100.5, 101, 120, 150]


def main():
    ap = argparse.ArgumentParser(description="Serielle CLI der Drohne testen")
    ap.add_argument("--port", default="COM11")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("heights", nargs="*", type=float, default=DEFAULT_HEIGHTS,
                    help="zu testende Zielhoehen (cm); leer = Standard-Sweep")
    args = ap.parse_args()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.15)
    except Exception as e:
        print(f"OPEN-ERROR {args.port}: {e}")
        print("-> Port belegt? Seriellen Monitor (PlatformIO/VS Code/PuTTY) auf "
              f"{args.port} schliessen. NICHT eigenmaechtig Prozesse beenden.")
        sys.exit(2)

    def drain(seconds):
        end = time.time() + seconds
        buf = b""
        while time.time() < end:
            d = ser.read(4096)
            if d:
                buf += d
        return buf.decode("utf-8", "replace")

    def cmd(s, wait=0.8):
        # zuerst evtl. haengende Teilzeile im Geraetepuffer per '\r' ausloesen
        ser.write(b"\r")
        time.sleep(0.15)
        ser.reset_input_buffer()
        ser.write(s.encode() + b"\r")
        ser.flush()
        return drain(wait)

    time.sleep(0.3)
    drain(0.4)

    print("########## help ##########")
    print(cmd("help", 1.2))

    print("########## getHeight (Ausgangswert) ##########")
    print(cmd("getHeight", 0.8))

    print("########## setHeight ohne Argument ##########")
    print(cmd("setHeight", 0.8))

    print(f"########## Hoehen-Sweep ({args.heights[0]} .. {args.heights[-1]}) ##########")
    results = []
    for h in args.heights:
        cmd(f"setHeight {h}", 0.5)
        getout = cmd("getHeight", 0.6)
        val = ""
        for line in getout.splitlines():
            if "targetHeightCm=" in line:
                val = line.strip().split("=", 1)[1]
        results.append((h, val))
        print(f"setHeight {h:>8} -> targetHeightCm={val}")

    ser.close()

    print("########## Zusammenfassung ##########")
    vals = [float(v) for _, v in results if v]
    if vals:
        print(f"beobachteter Wertebereich targetHeightCm: {min(vals):.2f} .. {max(vals):.2f}")
    else:
        print("keine getHeight-Werte gelesen (Port/Firmware pruefen)")
    print("fertig.")


if __name__ == "__main__":
    main()
