
# Helmivo — ESP32 smart helmet / accident detection
<img width="1179" height="1758" alt="9fdd92f9-58a5-4a0d-ae86-2f1d0d9601bf" src="https://github.com/user-attachments/assets/7d2c68a2-7a1f-4570-83fd-a27738588eaf" />
Arduino firmware for an ESP32-based **smart helmet** prototype: **MPU6050** motion sensing, **u-blox GPS** over UART, **Wi-Fi** (access point + optional home Wi-Fi), a **web dashboard**, **Telegram** alerts, and GPIO for a **relay (helmet lights)** and **buzzer**.

**Disclaimer:** This is experimental firmware, not a certified safety or medical device. Do not rely on it for life-critical decisions.

## Hardware

| Function | Connection |
|----------|------------|
| **MPU6050** | I2C SDA **GPIO21**, SCL **GPIO22** |
| **GPS (u-blox NMEA, 9600 baud)** | ESP32 RX **GPIO16** ← module TX; ESP32 TX **GPIO17** → module RX |
| **Relay (helmet lights)** | **GPIO14** (active **LOW** = lights on) |
| **Buzzer** | **GPIO25** |
| **Cancel / input button** | **GPIO13** (`INPUT_PULLUP`) |

Use a 3.3 V–compatible GPS module; level-shift if your module is 5 V UART.

## Software

- **Board:** ESP32 (Arduino-ESP32 core).
- **Dependencies:** Standard core libraries only (`Wire`, `WiFi`, `WebServer`, `Preferences`, `WiFiClientSecure`, `HTTPClient`). The MPU6050 is driven via raw I2C registers defined in the sketch.

Open the folder in Arduino IDE 2.x (or use the `.ino` with your usual ESP32 toolchain).

### Secrets (Telegram bot token)

1. Copy `secrets.h.example` to `secrets.h`.
2. Set `HELMIVO_BOT_TOKEN` to the token from [@BotFather](https://t.me/BotFather).

`secrets.h` is listed in `.gitignore` so it is not committed. You can also set or override the token from the web UI (stored in NVS).

## First run

1. Flash the sketch.
2. Connect a phone or laptop to the Wi-Fi access point:
   - **SSID:** `Helmivo`
   - **Password:** `Helmivo`
3. Open the device in a browser. The AP IP is usually **`http://192.168.4.1`** (check the serial log for `AP: ... IP:`).

### Optional: connect to your home Wi-Fi

Use the **Wi-Fi** page in the UI (`/wifi`). The device keeps the AP up for local access; STA credentials are saved and used after reboot.

### Telegram

Configure your **chat ID** (and optional bot token) via the **Telegram** page or `POST /api/set-telegram` (`application/x-www-form-urlencoded`: `chat`, optional `bot`).

## Web routes

| Path | Purpose |
|------|---------|
| `/` | Landing |
| `/wifi` | Configure STA Wi-Fi |
| `/telegram` | Telegram settings |
| `/dashboard` | Main dashboard |
| `/api/status` | JSON status |
| `/api/set-telegram` | Save chat / bot token |
| `/api/cancel-alert` | Cancel active alert flow |
| `/api/serial-log` | Buffered serial lines; optional `?since=N` |
| `/tester` | Tester panel UI |

### Tester / debug API

| Method | Path | Notes |
|--------|------|--------|
| `POST` | `/api/test/accident` | Inject a fake accident (cancel window) |
| `POST` | `/api/test/flashlight` | Exercise relay ~5 s |
| `POST` | `/api/test/buzzer` | Buzzer ~3 s |
| `GET` | `/api/test/posture` | MPU: G, pitch, roll, tilt, gyro |
| `GET` | `/api/test/gps` | GPS fix, lat/lon, satellites, age |

## Serial monitor

At your configured UART baud rate:

- **`c`** — cancel Telegram countdown / clear cancel window where applicable  
- **`r`** — reset impact / accident stats  
- **`t`** — test countdown behaviour  

Startup log prints AP SSID/IP and mentions the dashboard on the AP (e.g. `http://192.168.4.1/dashboard`).

## Repository layout

- `esp32_smart_accident_detection.ino` — main firmware  
- `secrets.h.example` — template for `secrets.h`  
- `.gitignore` — excludes `secrets.h`  

---

*The repo folder name is `esp32_smart_accident_detection`; the firmware refers to the product as **Helmivo**.*
