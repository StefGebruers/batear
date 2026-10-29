#!/usr/bin/env python3
"""
configure_wifi.py — Stuur Wi-Fi en MQTT instellingen naar BatEar NVS via USB serieel
en toon daarna de live seriële monitor (met automatische herverbinding na USB reboot).
"""
import sys
import time
import serial


def open_serial_retry(port: str, baud: int = 115200, timeout_s: float = 10.0) -> serial.Serial:
    deadline = time.time() + timeout_s
    last_err = None
    while time.time() < deadline:
        try:
            return serial.Serial(port, baud, timeout=0.1)
        except Exception as e:
            last_err = e
            time.sleep(0.25)
    raise RuntimeError(f"Kon {port} niet openen: {last_err}")


def send_cmd(ser: serial.Serial, cmd: str, wait_s: float = 0.45) -> None:
    ser.write((cmd + "\r\n").encode("utf-8"))
    ser.flush()
    end = time.time() + wait_s
    while time.time() < end:
        try:
            if ser.in_waiting:
                data = ser.read(ser.in_waiting)
                sys.stdout.write(data.decode("utf-8", errors="replace"))
                sys.stdout.flush()
        except Exception:
            break
        time.sleep(0.02)


def main() -> int:
    if len(sys.argv) < 2:
        print("Gebruik: configure_wifi.py <PORT> [WIFI_SSID] [WIFI_PASS] [MQTT_USER] [MQTT_PASS]")
        return 1

    port = sys.argv[1]
    wifi_ssid = sys.argv[2] if len(sys.argv) > 2 else ""
    wifi_pass = sys.argv[3] if len(sys.argv) > 3 else ""
    mqtt_user = sys.argv[4] if len(sys.argv) > 4 else ""
    mqtt_pass = sys.argv[5] if len(sys.argv) > 5 else ""

    print(f"Verbinden met {port} op 115200 baud...")
    ser = open_serial_retry(port)

    try:
        # Wacht tot de ESP32-S3 na het flashen is opgestart en de console klaar is
        start = time.time()
        while time.time() - start < 4.0:
            try:
                if ser.in_waiting:
                    chunk = ser.read(ser.in_waiting).decode("utf-8", errors="replace")
                    sys.stdout.write(chunk)
                    sys.stdout.flush()
                    if "Console ready" in chunk or "batear>" in chunk or "No WiFi SSID" in chunk:
                        time.sleep(0.2)
                        break
            except Exception:
                time.sleep(0.2)
                ser = open_serial_retry(port)
            time.sleep(0.05)

        if wifi_ssid:
            print("\n=== Wi-Fi & MQTT instellingen opslaan in NVS ===")
            send_cmd(ser, "", wait_s=0.2)
            esc_ssid = wifi_ssid.replace('"', '\\"')
            esc_pass = wifi_pass.replace('"', '\\"')
            send_cmd(ser, f'set wifi_ssid "{esc_ssid}"')
            send_cmd(ser, f'set wifi_pass "{esc_pass}"')
            send_cmd(ser, 'set mqtt_url "mqtt://192.168.1.142:1883"')
            if mqtt_user:
                esc_muser = mqtt_user.replace('"', '\\"')
                esc_mpass = mqtt_pass.replace('"', '\\"')
                send_cmd(ser, f'set mqtt_user "{esc_muser}"')
                send_cmd(ser, f'set mqtt_pass "{esc_mpass}"')
            send_cmd(ser, "show", wait_s=0.6)
            print("\n=== Herstarten om Wi-Fi te verbinden ===")
            send_cmd(ser, "reboot", wait_s=0.3)
            try:
                ser.close()
            except Exception:
                pass
            time.sleep(1.2)
            ser = open_serial_retry(port)

        print("\n=== Live Seriële Monitor (druk op Ctrl+C om te stoppen) ===")
        while True:
            try:
                if ser.in_waiting:
                    data = ser.read(ser.in_waiting)
                    sys.stdout.write(data.decode("utf-8", errors="replace"))
                    sys.stdout.flush()
                time.sleep(0.03)
            except (serial.SerialException, OSError):
                # USB-Serial/JTAG re-enumeratie na reboot
                try:
                    ser.close()
                except Exception:
                    pass
                time.sleep(0.5)
                ser = open_serial_retry(port)
    except KeyboardInterrupt:
        print("\nMonitor gestopt.")
    finally:
        try:
            ser.close()
        except Exception:
            pass

    return 0


if __name__ == "__main__":
    sys.exit(main())
