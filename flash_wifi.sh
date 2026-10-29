#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

WIFI_SSID="${1:-}"
WIFI_PASS="${2:-}"
MQTT_USER="${3:-}"
MQTT_PASS="${4:-}"

# Zoek automatisch de juiste seriële poort (/dev/ttyACM0 voor Heltec V4 S3, of /dev/ttyUSB0)
if [ -e "/dev/ttyACM0" ]; then
    PORT="/dev/ttyACM0"
elif [ -e "/dev/ttyUSB0" ]; then
    PORT="/dev/ttyUSB0"
else
    echo "Fout: Geen ESP32 gevonden op /dev/ttyACM0 of /dev/ttyUSB0."
    exit 1
fi

if [ ! -w "$PORT" ]; then
    echo "Geef lees-/schrijfrechten op $PORT..."
    sudo chmod a+rw "$PORT"
fi

echo "=== 1. BatEar Wi-Fi Detector compileren & flashen naar $PORT ==="
"$ROOT_DIR/.venv/bin/pio" run -d "$SCRIPT_DIR" -t upload --upload-port "$PORT"

echo ""
echo "=== 2. Configureren & Monitor starten op $PORT ==="
"$ROOT_DIR/.venv/bin/python" "$SCRIPT_DIR/configure_wifi.py" "$PORT" "$WIFI_SSID" "$WIFI_PASS" "$MQTT_USER" "$MQTT_PASS"

