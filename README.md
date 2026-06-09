# ESP32 Desk Radar (GC9A01 + ADSB.fi + BLE Config)

This project turns an ESP32 + GC9A01 round TFT into a mini air-traffic radar display.

## What it does

- Connects to Wi-Fi
- Pulls nearby aircraft from the free ADSB.fi API
- Dynamically fetches nearby airports using the OpenStreetMap Overpass API
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
- Client_id: `7a0b1010-25be-45b3-8a2f-d5e9f53c1010` *(Legacy/Ignored)*
- Client_secret: `7a0b1011-25be-45b3-8a2f-d5e9f53c1011` *(Legacy/Ignored for ADSB.fi)*
- Command: `7a0b1008-25be-45b3-8a2f-d5e9f53c1008`
- Status (read/notify): `7a0b1009-25be-45b3-8a2f-d5e9f53c1009`

Command values:

- `save` -> save settings
- `apply` -> save + reconnect Wi-Fi immediately
- `auth` -> *(Legacy)* replies that auth is not needed for ADSB.fi
- `verbose` or `verbose on` -> very talky debug logs (full API payloads)
- `verbose off` -> normal debug logs
- `airports on` -> turns on airport drawing (default)
- `airports off` -> turns off airport drawing
- `clearwifi` -> erase Wi-Fi creds
- `reboot` -> restart ESP32

Typical first-time setup:

1. Write SSID
2. Write password
3. Write latitude/longitude/radius/speed cutoff
4. Write command `apply`
5. Read status characteristic

---

## 4) ADSB.fi API Details

This project uses the free and open [ADSB.fi v3 API](https://adsb.fi/). Authentication is no longer required, meaning you will not be rate-limited the way anonymous OpenSky users are. 

**Note on Range:** The ADSB.fi API enforces a maximum search radius of 250 Nautical Miles. The code automatically handles the conversion from your configured km radius, capping requests at ~463 km to prevent API errors.

Radar filtering also enriches aircraft with ADSBDB (`api.adsbdb.com`) by Mode-S and callsign. Only major/interesting aircraft ICAO types are shown (jet airliners, heavy cargo, large military, fighters). If ADSBDB lookup is unavailable, fewer aircraft may be displayed until lookups succeed.

Aircraft tags include route IATA codes on a separate line (for example `FRA-AYT`) when available.

---

## 5) Dynamic Airports (OpenStreetMap)

The radar dynamically fetches nearby airports based on your configured coordinates and radius using the free **OpenStreetMap Overpass API**. No API key is required.

The radar caches the local airports in memory to save bandwidth. It will only call the API again if you change your radar radius, or if your center coordinates move by more than 1 kilometer.

---

## 6) Tuning

In [src/main.cpp](src/main.cpp):

- `FETCH_INTERVAL_MS` controls network fetch rate
- `MAX_AIRCRAFT` caps rendered aircraft (default 40 to save memory)
- `MAX_ADSB_CACHE` caps the number of aircraft cached for trails and routes
- `MAX_TRAIL` caps the length of the trail behind each aircraft
- `MAX_DYNAMIC_REGIONS` caps the number of screen regions updated per frame
- `PIN_TFT_*` configures display pins

---

## 7) Legal note

ADSB.fi open data usage is subject to their community terms and guidelines.