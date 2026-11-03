<div align="center">
  <img src="icon.png" alt="Batear Logo" width="200"/>
  
  <h1>Batear</h1>
  <p><strong>Ultra-low-cost, edge-only acoustic drone detector on ESP32-S3 with 4-microphone 3D Direction of Arrival (DoA), GPS positioning, Wi-Fi / Ethernet / LoRa telemetry, and Home Assistant integration.</strong></p>

<p align="center">
  <a href="https://opencollective.com/batear"><img src="https://img.shields.io/opencollective/all/batear?label=Sponsors&color=blue" alt="Sponsors"></a>
  <a href="https://opencollective.com/batear"><img src="https://img.shields.io/badge/501(c)(3)-Tax%20Deductible-brightgreen" alt="Tax Deductible"></a>
  <a href="https://hackaday.com/2026/03/23/acoustic-drone-detection-on-the-cheap-with-esp32/"><img src="https://img.shields.io/badge/Featured%20on-Hackaday-black?logo=hackaday" alt="Featured on Hackaday"></a>
  <a href="https://github.com/batear-io/batear/stargazers"><img src="https://img.shields.io/github/stars/batear-io/batear?style=flat-square" alt="Stars"></a>
</p>

  <br>
  <p><em>"Built for defense, hoping it becomes unnecessary. We believe in a world where no one needs to fear the sky."</em></p>
</div>

---

## 🌟 Key Highlights & Capabilities

Batear turns an ultra-affordable **ESP32-S3** microcontroller into a standalone acoustic drone detection radar:

* **Real-time Rotor Harmonic Detection**: Performs 1024-point complex FFTs (16 kHz, 64 ms frames) using SIMD vector acceleration to isolate multi-harmonic rotor blade signatures in the 150 Hz – 1200 Hz spectrum.
* **4-Microphone 3D Direction of Arrival (DoA)**:
  * Computes **360° Azimuth (`doa_deg`)** and **0°–90° Elevation (`doa_elev_deg`)**.
  * Outputs 8-point compass cardinal directions (`N`, `NE`, `E`, `SE`, `S`, `SW`, `W`, `NW`).
  * Phase-difference cross-correlation (TDOA / GCC-PHAT) across North/South and East/West microphone axes.
* **Automatic Startup Calibration**: Balances channel trims across all 4 microphones and measures the local ambient acoustic baseline in real-time.
* **GPS / GNSS Integration**: Onboard NMEA GPS parser feeds real-time latitude, longitude, altitude, and synchronized ISO-8601 UTC timestamps into every detection event.
* **Battery & Power Management**: Gated ADC resistor divider with 1S LiPo/Li-ion discharge curve interpolation (`battery_mv` and `battery_pct`).
* **Direct Home Assistant Integration**: Native MQTT Auto-Discovery creates entities for Drone Alarm, Confidence, Azimuth, Elevation, Cardinal Direction, GPS coordinates, and Battery status.
* **Interactive Serial Console**: Real-time inspection and runtime tuning (`sensitivity`, `mic_gain`, `mic_spacing`, `north_offset`, `wifi`, `mqtt`) stored in non-volatile flash (NVS).

---

## 🏗️ Supported Operating Roles

The same codebase can be built for any of the three operational roles:

```
                          ┌────────────────────────────────────────────────────────┐
                          │               BATTERY-POWERED WI-FI DETECTOR           │
                          │   Heltec WiFi LoRa 32 V4 (ESP32-S3)                    │
                          │   • 4× MEMS Mics (North, South, East, West)            │
                          │   • 3D Direction of Arrival (Azimuth + Elevation)      │
                          │   • NEO GPS Module (Position + UTC Clock Sync)         │
                          │   • SSD1306 OLED Screen + Alarm LED                    │
                          │   • ADC Gated Battery Monitor (mV + %)                 │
                          └──────────────────────────┬─────────────────────────────┘
                                                     │ Wi-Fi (MQTT JSON)
                                                     ▼
┌──────────────────────────────────────┐  MQTT   ┌────────────────────────────────┐
│      PERMANENT WIRED DETECTOR        │ ──────► │         HOME ASSISTANT         │
│  LILYGO T-ETH-Lite S3 (PoE/Ethernet) │         │       (Auto-Discovery)         │
└──────────────────────────────────────┘         │                                │
                                                 │ • Binary Safety Sensor (Drone) │
┌──────────────────────────────────────┐         │ • Confidence Level (%)         │
│      LORA DETECTOR & GATEWAY         │  LoRa   │ • Azimuth Angle (0°–360°)      │
│  Heltec V3/V4 (AES-128-GCM encrypted)│ ──────► │ • Elevation Angle (0°–90°)     │
└──────────────────────────────────────┘         │ • Battery Monitor (%) & (V)    │
                                                 │ • GPS Map Coordinates         │
                                                 └────────────────────────────────┘
```

---

## 🎙️ 4-Microphone Cross-Array & Hardware Setup

The 4-microphone planar cross-array uses two shared I2S peripheral lines (Dual I2S DIN channels) for simultaneous phase-coherent sampling:

```
               [Mic 0: North]
             (DIN=GPIO6, SEL=GND)
                      ▲
                      │  mic_spacing / 2
                      │
[Mic 3: West] ◄───────┼───────► [Mic 2: East]
(DIN=GPIO7, SEL=3V3)  │         (DIN=GPIO7, SEL=GND)
                      │  mic_spacing / 2
                      ▼
               [Mic 1: South]
             (DIN=GPIO6, SEL=3V3)
```

### Pin Mapping (Heltec WiFi LoRa 32 V4 / V3)

| Function | ESP32-S3 Pin | Connected Peripheral / Signal |
| :--- | :--- | :--- |
| **I2S BCLK** | `GPIO 4` | Bit Clock (all 4 microphones) |
| **I2S WS (LRCLK)** | `GPIO 5` | Word Select / Frame Clock (all 4 microphones) |
| **I2S DIN (Pair 1)** | `GPIO 6` | North Mic (`SEL=GND`, Ch0) & South Mic (`SEL=3V3`, Ch1) |
| **I2S DIN (Pair 2)** | `GPIO 7` | East Mic (`SEL=GND`, Ch2) & West Mic (`SEL=3V3`, Ch3) |
| **GPS UART RX** | `GPIO 43` | GPS Module TX line (9600 baud NMEA) |
| **GPS UART TX** | `GPIO 44` | GPS Module RX line |
| **Battery ADC** | `GPIO 1` | Resistor divider tap (`ADC1_CH0`, 4.9× ratio) |
| **Battery Gate** | `GPIO 37` | Active-low P-FET gate (eliminates quiescent drain) |
| **OLED SDA / SCL** | `GPIO 17 / 18`| Onboard 0.96" SSD1306 Display |
| **VEXT Control** | `GPIO 36` | Active-low power gate for OLED display |
| **Alarm LED** | `GPIO 35` | Onboard visual alarm indicator |

---

## ⚡ Quick Start & Flashing

### Wi-Fi Detector (Heltec V4)
Clone the repository and run the automated build & flash script:

```bash
cd batear
./flash_wifi.sh
```

*(Alternatively, use `./flash_wifi.sh /dev/ttyACM1` if your board is assigned to a different USB port).*

### Configuration Console (USB Serial / JTAG)
Connect to the serial console at `115200` baud (e.g. via `pio device monitor` or `minicom`):

```text
batear> show
--- Batear Configuration ---
[lora_cfg]
  device_id   = 1
  sensitivity = 3  (active, 1=Strict .. 5=Max)
  mic_gain    = 2x (active, 2x=+6dB default)
  mic_spacing = 100 mm
  north_offset= 0 deg
[mic_cal]
  Mic0 Noord:  trim = 1.00x (+0.0 dB)
  Mic1 Zuid:   trim = 0.98x (-0.2 dB)
  Mic2 Oost:   trim = 1.03x (+0.3 dB)
  Mic3 West:   trim = 0.99x (-0.1 dB)
  ambient_rms = 0.00042
[power]
  voltage     = 4050 mV (4.05 V)
  percentage  = 88 %
[wifi_det_cfg]
  wifi_ssid   = "MyNetwork"
  mqtt_url    = "mqtt://192.168.1.76:1883"
  device_id   = "det01"
```

---

## ⌨️ Console Commands

All parameters configured via `set` are immediately committed to non-volatile flash storage (**NVS**) and persist across power cuts and reboots.

| Command | Arguments | Description | Applies |
| :--- | :--- | :--- | :--- |
| `show` | - | Displays active configuration, trims, baseline, battery, and MQTT settings | Immediate |
| `set sensitivity` | `1` .. `5` | Detection sensitivity (`1` = strict, `3` = balanced, `5` = maximum range) | **Live** |
| `set mic_gain` | `1` .. `32` | Digital pre-amplification gain (`2` = +6dB, `4` = +12dB) | **Live** |
| `set mic_spacing` | `20` .. `500`| Distance in millimeters between opposing microphones (default `100`) | **Live** |
| `set north_offset`| `0` .. `359` | Orientation offset in degrees from Mic 0 to true North | **Live** |
| `set debug` | `0` or `1` | Enable/disable 1 Hz live audio spectrum diagnostics log | **Live** |
| `set wifi_ssid` | `<name>` | Wi-Fi network SSID | After reboot |
| `set wifi_pass` | `<pass>` | Wi-Fi network WPA2 password | After reboot |
| `set mqtt_url` | `mqtt://<host>:<port>` | MQTT broker address (default `mqtt://192.168.1.76:1883`) | After reboot |
| `set mqtt_device_id`| `<id>` | Unique node identifier for MQTT topics (e.g. `det01`) | After reboot |
| `cal` | - | Displays microphone trims and current background noise floor | Immediate |
| `cal run` | - | Triggers a 2.5s live acoustic recalibration of all 4 microphones | **Live** |
| `gps` | - | Displays current GPS satellite fix, coordinates, altitude, and HDOP | Immediate |
| `gps fix` | - | Forces GPS module wake-up and immediate satellite re-acquisition | Immediate |
| `reboot` | - | Restarts the ESP32-S3 microcontroller | Immediate |

---

## 📡 MQTT Telemetry & Home Assistant Integration

Status updates are published to `batear/nodes/<node_id>/status`.

### JSON Payload Schema

```json
{
  "drone_detected": true,
  "detector_id": 1,
  "rms_db": 64,
  "f0_bin": 22,
  "seq": 463,
  "confidence": 0.8950,
  "doa_deg": 262,
  "doa_elev_deg": 21,
  "doa_cardinal": "W",
  "battery_pct": 85,
  "battery_mv": 4020,
  "lat": 51.106318,
  "lon": 5.009883,
  "alt_m": 45.7,
  "timestamp_iso": "2026-11-03T09:02:45Z",
  "timestamp": 1793696565
}
```

* **Heartbeat Telemetry**: When idle, the detector publishes a diagnostic status update every **60 seconds** to ensure battery percentage, battery voltage, and GPS coordinates stay refreshed in Home Assistant.
* **Home Assistant Entities**:
  * `binary_sensor.batear_<id>_drone`: Primary alarm (safety device class).
  * `sensor.batear_<id>_confidence`: Confidence level (0–100%).
  * `sensor.batear_<id>_doa_deg`: Horizontal Azimuth angle in degrees (°).
  * `sensor.batear_<id>_doa_elev_deg`: Elevation angle in degrees (°).
  * `sensor.batear_<id>_doa_cardinal`: 8-point cardinal compass text (`N`, `NE`, `E`, etc.).
  * `sensor.batear_<id>_battery`: Battery level (%) with dynamic battery icon.
  * `sensor.batear_<id>_battery_mv`: Battery voltage (V).
  * `sensor.batear_<id>_latitude`, `longitude`, `altitude`: GPS position.
  * `sensor.batear_<id>_timestamp`: Synced UTC time.

---

## 📊 System Resource Utilization

* **Flash Utilization**: ~1.14 MB out of 8 MB / 16 MB (**~14%**).
* **RAM Utilization**: ~123 KB statically allocated, leaving **~218 KB free heap** (**~64% free**).
* **CPU Load**: Dual-core 240 MHz runs at **< 5% average utilization** (Core 1 handles 15.6 FFTs/sec in ~1.4 ms per 64 ms frame; Core 0 handles Wi-Fi/MQTT event loops).
* **Power Draw & Battery Life**:
  * Active continuous consumption: ~140–160 mA @ 3.7V (ESP32-S3 + Wi-Fi STA + GPS + OLED + 4 Mics).
  * ~5.5–6 hours on 1000 mAh LiPo.
  * ~11–12 hours on 2000 mAh Li-ion (18650).
  * ~17–20 hours on 3000–3500 mAh Li-ion (21700).

---

## 🛡️ License & Mission

Batear is open-source hardware and software built to protect civilians, families, and critical infrastructure. 
Visit **[batear.io](https://docs.batear.io)** or support the mission on **[Open Collective](https://opencollective.com/batear)**.
