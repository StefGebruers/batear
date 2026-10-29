#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

if [ $# -lt 1 ]; then
    echo "Gebruik: ./set_sensitivity.sh <1..5> [mic_gain 1..32]"
    echo "  1 = Streng (enkel luid & dichtbij)"
    echo "  2 = Conservatief"
    echo "  3 = Gebalanceerd (standaard)"
    echo "  4 = Gevoelig"
    echo "  5 = Maximaal"
    exit 1
fi

SENS="$1"
GAIN="${2:-}"

if [ -e "/dev/ttyACM0" ]; then
    PORT="/dev/ttyACM0"
elif [ -e "/dev/ttyUSB0" ]; then
    PORT="/dev/ttyUSB0"
else
    echo "Fout: Geen ESP32 gevonden op /dev/ttyACM0 of /dev/ttyUSB0."
    exit 1
fi

"$ROOT_DIR/.venv/bin/python" - "$PORT" "$SENS" "$GAIN" <<'EOF'
import sys, time, serial

port, sens, gain = sys.argv[1], sys.argv[2], sys.argv[3]
ser = serial.Serial(port, 115200, timeout=0.1)

def send(cmd):
    ser.write((cmd + "\r\n").encode("utf-8"))
    ser.flush()
    end = time.time() + 0.4
    while time.time() < end:
        if ser.in_waiting:
            sys.stdout.write(ser.read(ser.in_waiting).decode("utf-8", errors="replace"))
            sys.stdout.flush()
        time.sleep(0.02)

send("")
send(f"set sensitivity {sens}")
if gain:
    send(f"set mic_gain {gain}")
send("show")
ser.close()
EOF
