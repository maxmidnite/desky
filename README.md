# ESP32 Desk Radar (GC9A01 + OpenSky + BLE Config)

This project turns an ESP32 + GC9A01 round TFT into a mini air-traffic radar display.

## What it does

- Connects to Wi-Fi
- Pulls nearby aircraft from OpenSky API
- Draws radar scope with sweep + aircraft blips on a 240x240 circular display
- Lets you configure Wi-Fi + radar area over BLE (Bluetooth Low Energy)
- Saves config to ESP32 flash (NVS)

---

## 1) VS Code setup (PlatformIO)

1. Install **Visual Studio Code**.
2. Install extension: **PlatformIO IDE**.
3. Open this folder in VS Code.
4. Connect ESP32 with USB.
5. In PlatformIO, click **Build** then **Upload**.
6. Open serial monitor at **115200** baud.

The project is already configured through [platformio.ini](platformio.ini).

---

## 2) Wiring (ESP32 -> GC9A01 SPI display)

Use **3.3V logic only**.

| GC9A01 pin | ESP32 pin |
|---|---|
| VCC | 3V3 |
| GND | GND |
| SCL / CLK | GPIO18 |
| SDA / DIN / MOSI | GPIO23 |
| CS | GPIO5 |
| DC | GPIO2 |
| RST | GPIO4 |

> Notes:
> - Some modules label pins differently (`SCL`=`SCK`, `SDA`=`MOSI`).
> - If your display has no `BL`/`LED` pin, that is normal. Backlight is internally tied and no extra wire is needed.
> - If your display *does* expose `BL`/`LED`, connect it to a free GPIO (example `GPIO15`) and set `PIN_TFT_BL` in [src/main.cpp](src/main.cpp).
> - If your board uses other SPI pins, update constants in [src/main.cpp](src/main.cpp).

---

## 3) BLE configuration from phone

Use a BLE app like **nRF Connect**.

- Device name: `DeskRadar-XXXX`
- Service UUID: `7a0b1001-25be-45b3-8a2f-d5e9f53c1001`

Write UTF-8 text to characteristics:

- SSID: `7a0b1002-25be-45b3-8a2f-d5e9f53c1002`
- Password: `7a0b1003-25be-45b3-8a2f-d5e9f53c1003`
- Center latitude: `7a0b1004-25be-45b3-8a2f-d5e9f53c1004`
- Center longitude: `7a0b1005-25be-45b3-8a2f-d5e9f53c1005`
- Radius km: `7a0b1006-25be-45b3-8a2f-d5e9f53c1006`
- Speed cutoff kts: `7a0b1007-25be-45b3-8a2f-d5e9f53c1007`
- OpenSky client_id: `7a0b1010-25be-45b3-8a2f-d5e9f53c1010`
- OpenSky client_secret: `7a0b1011-25be-45b3-8a2f-d5e9f53c1011`
- Command: `7a0b1008-25be-45b3-8a2f-d5e9f53c1008`
- Status (read/notify): `7a0b1009-25be-45b3-8a2f-d5e9f53c1009`

Command values:

- `save` -> save settings
- `apply` -> save + reconnect Wi-Fi immediately
- `auth` -> request/refresh OpenSky auth token now
- `verbose` or `verbose on` -> very talky debug logs (full API payloads)
- `verbose off` -> normal debug logs
- `clearwifi` -> erase Wi-Fi creds
- `reboot` -> restart ESP32

Typical first-time setup:

1. Write SSID
2. Write password
3. Write OpenSky `client_id`
4. Write OpenSky `client_secret`
5. Write latitude/longitude/radius/speed cutoff
6. Write command `apply`
7. Read status characteristic

---

## 4) OpenSky limits

OpenSky public API can rate limit anonymous users. If aircraft count is often zero, this can be the reason.

Radar filtering also enriches aircraft with ADSBDB (`api.adsbdb.com`) by Mode-S and callsign.
Only major/interesting aircraft ICAO types are shown (jet airliners, heavy cargo, large military, fighters).
If ADSBDB lookup is unavailable, fewer aircraft may be displayed until lookups succeed.

Aircraft tags now include route IATA codes on a separate line (for example `FRA-AYT`) when available.

---

## 5) Tuning

In [src/main.cpp](src/main.cpp):

- `FETCH_INTERVAL_MS` controls network fetch rate
- `MAX_AIRCRAFT` caps rendered aircraft
- `PIN_TFT_*` configures display pins

---

## 6) Legal note

OpenSky data usage is subject to OpenSky terms and fair use limits.
