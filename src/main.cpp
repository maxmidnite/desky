#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <NimBLEDevice.h>
#include <SPI.h>
#include <time.h>
#include <Adafruit_GFX.h>
#include <Adafruit_GC9A01A.h>

// -------------------- Pin map (change if needed) --------------------
constexpr int PIN_TFT_SCK = 18;
constexpr int PIN_TFT_MOSI = 23;
constexpr int PIN_TFT_CS = 5;
constexpr int PIN_TFT_DC = 2;
constexpr int PIN_TFT_RST = 4;
constexpr int PIN_TFT_BL = -1; // Set to GPIO if your module exposes BL/LED, else keep -1

// -------------------- Radar display geometry --------------------
constexpr int SCREEN_W = 240;
constexpr int SCREEN_H = 240;
constexpr int CENTER_X = 120;
constexpr int CENTER_Y = 120;
constexpr int RADAR_RADIUS_PX = 108;

// -------------------- App behavior --------------------
constexpr uint32_t FETCH_INTERVAL_MS = 1500;
constexpr uint32_t BLIP_REFRESH_INTERVAL_MS = 1500;
constexpr int MAX_AIRCRAFT = 40;
constexpr int MAX_AIRPORTS = 20;
constexpr int MAX_TAGS_ON_SCREEN = 18;
constexpr int MAX_ADSB_CACHE = 40;
constexpr uint32_t ADSB_LOOKUP_SPACING_MS = 1200;

// -------------------- OTA Update settings --------------------
constexpr float CURRENT_VERSION = 0.9f;
const char* FW_VERSION_URL = "https://raw.githubusercontent.com/maxmidnite/desky/main/release/version.json";
const char* FW_BIN_URL = "https://raw.githubusercontent.com/maxmidnite/desky/main/release/firmware.bin";

Adafruit_GC9A01A tft(PIN_TFT_CS, PIN_TFT_DC, PIN_TFT_RST);
GFXcanvas16 canvas(SCREEN_W, SCREEN_H);
Preferences prefs;

struct Config {
  String ssid;
  String pass;
  float centerLat;
  float centerLon;
  float radiusKm;
  float speedCutoffKts;
  bool showAirports;
  String apiClientId;
  String apiClientSecret;
};

Config cfg;

struct Aircraft {
  String callsign;
  String modeS;
  String icaoType;
  String routeIata;
  float lat;
  float lon;
  float trackDeg;
  float velocity;
  float altitudeM;
};

struct DynamicAirport {
  char iata[5]; // 4 chars + null terminator for safety
  float lat;
  float lon;
};

struct AirportCache {
  int count;
  DynamicAirport airports[MAX_AIRPORTS];
  float savedLat;
  float savedLon;
  float savedRad;
};

DynamicAirport cachedAirports[MAX_AIRPORTS];
int cachedAirportCount = 0;

Aircraft aircraft[MAX_AIRCRAFT];
int aircraftCount = 0;

#define NUM_COLORS 16
uint16_t planeColors[NUM_COLORS];
int nextColorIdx = 0;

void setupColors() {
  uint32_t hexColors[NUM_COLORS] = {
    0x9ad48b, 0x448f30, 0x236a10, 0x6cb359, 0x69A097, 0x246c60, 0x0c4f44, 0x43877b,
    0x78131f, 0xa33643, 0xf29ea8, 0xcc6571, 0xd5956a, 0x7d3e14, 0xaa6739, 0xfcc8a5
  };
  randomSeed(esp_random());
  for (int i = 0; i < NUM_COLORS; i++) {
    uint8_t r = (hexColors[i] >> 16) & 0xFF;
    uint8_t g = (hexColors[i] >> 8) & 0xFF;
    uint8_t b = hexColors[i] & 0xFF;
    planeColors[i] = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
  }
  for (int i = 0; i < NUM_COLORS; i++) {
    int swapIdx = random(NUM_COLORS);
    uint16_t temp = planeColors[i];
    planeColors[i] = planeColors[swapIdx];
    planeColors[swapIdx] = temp;
  }
}

uint32_t lastFetchMs = 0;
uint32_t lastBlipDrawMs = 0;
bool wifiConnected = false;
bool canvasReady = false;
bool dynamicDirty = true;
bool firstFrame = true;
bool bleClientConnected = false;
int lastHttpCode = 0;
uint32_t successfulFetchCount = 0;
uint32_t fetchEpoch = 0;
String lastStatusText = "Boot";
uint32_t lastNoWiFiStatusMs = 0;
uint32_t lastWiFiRetryMs = 0;
uint32_t lastAdsbLookupMs = 0;

constexpr int MAX_TRAIL = 20;

struct Position {
  float lat;
  float lon;
};

struct AdsbCacheEntry {
  bool used;
  String modeS;
  String routeIata;
  bool lookedUp;
  bool hasRoute;
  bool hasLabelAngle;
  float currentLabelAngle;
  float targetLabelAngle;
  uint32_t lastLookupMs;

  uint16_t color;
  Position trail[MAX_TRAIL];
  int trailHead;
  int trailCount;
  uint32_t lastSeenMs;
};

AdsbCacheEntry adsbCache[MAX_ADSB_CACHE];

void initAdsbCache() {
  for (int i = 0; i < MAX_ADSB_CACHE; i++) {
    adsbCache[i].used = false;
    adsbCache[i].modeS = "";
    adsbCache[i].routeIata = "";
    adsbCache[i].lookedUp = false;
    adsbCache[i].hasRoute = false;
  }
}

struct RectRegion {
  int16_t x;
  int16_t y;
  int16_t w;
  int16_t h;
};

constexpr int MAX_DYNAMIC_REGIONS = 150;
RectRegion prevDynamicRegions[MAX_DYNAMIC_REGIONS];
int prevDynamicRegionCount = 0;
RectRegion currentDynamicRegions[MAX_DYNAMIC_REGIONS];
int currentDynamicRegionCount = 0;

float lastAptLat = -999.0f;
float lastAptLon = -999.0f;
float lastAptRad = -1.0f;

void addCurrentRegion(int x, int y, int w, int h) {
  if (currentDynamicRegionCount >= MAX_DYNAMIC_REGIONS) return;
  currentDynamicRegions[currentDynamicRegionCount++] = {(int16_t)x, (int16_t)y, (int16_t)w, (int16_t)h};
}

// BLE UUIDs (custom service)
static NimBLEUUID RADAR_SERVICE_UUID("7a0b1001-25be-45b3-8a2f-d5e9f53c1001");
static NimBLEUUID SSID_CHAR_UUID("7a0b1002-25be-45b3-8a2f-d5e9f53c1002");
static NimBLEUUID PASS_CHAR_UUID("7a0b1003-25be-45b3-8a2f-d5e9f53c1003");
static NimBLEUUID LAT_CHAR_UUID("7a0b1004-25be-45b3-8a2f-d5e9f53c1004");
static NimBLEUUID LON_CHAR_UUID("7a0b1005-25be-45b3-8a2f-d5e9f53c1005");
static NimBLEUUID RADIUS_CHAR_UUID("7a0b1006-25be-45b3-8a2f-d5e9f53c1006");
static NimBLEUUID SPEED_CHAR_UUID("7a0b1007-25be-45b3-8a2f-d5e9f53c1007");
static NimBLEUUID CLIENT_ID_CHAR_UUID("7a0b1010-25be-45b3-8a2f-d5e9f53c1010");
static NimBLEUUID CLIENT_SECRET_CHAR_UUID("7a0b1011-25be-45b3-8a2f-d5e9f53c1011");
static NimBLEUUID CMD_CHAR_UUID("7a0b1008-25be-45b3-8a2f-d5e9f53c1008");
static NimBLEUUID STATUS_CHAR_UUID("7a0b1009-25be-45b3-8a2f-d5e9f53c1009");

NimBLECharacteristic* statusChar = nullptr;
bool verboseLogging = false;

void publishStatus(const String& text);
String urlEncode(const String& in);
int findAdsbCacheIndex(const String& modeS);
int reserveAdsbCacheIndex();
bool rectsOverlap(const RectRegion& a, const RectRegion& b);
int metersToFlightLevel(float meters);
AdsbCacheEntry* getOrAllocateCacheEntry(const String& modeS);

void debugLog(const String& msg) {
  Serial.println("[DBG] " + msg);
}

String wifiStatusToText(wl_status_t s) {
  switch (s) {
    case WL_NO_SHIELD: return "NO_SHIELD";
    case WL_IDLE_STATUS: return "IDLE";
    case WL_NO_SSID_AVAIL: return "NO_SSID";
    case WL_SCAN_COMPLETED: return "SCAN_DONE";
    case WL_CONNECTED: return "CONNECTED";
    case WL_CONNECT_FAILED: return "CONNECT_FAILED";
    case WL_CONNECTION_LOST: return "CONNECTION_LOST";
    case WL_DISCONNECTED: return "DISCONNECTED";
    default: return "UNKNOWN(" + String((int)s) + ")";
  }
}

void printConfigSummary() {
  debugLog("Config SSID=" + (cfg.ssid.isEmpty() ? String("<empty>") : cfg.ssid));
  debugLog("Config PASS_LEN=" + String(cfg.pass.length()));
  debugLog("Config LAT=" + String(cfg.centerLat, 5) + " LON=" + String(cfg.centerLon, 5) + " R=" + String(cfg.radiusKm, 1) + "km SPD>" + String(cfg.speedCutoffKts, 0) + "kts");
  debugLog("Config API_CLIENT_ID=" + (cfg.apiClientId.isEmpty() ? String("<empty>") : cfg.apiClientId));
  debugLog("Config API_CLIENT_SECRET_LEN=" + String(cfg.apiClientSecret.length()));
}

float degToRad(float d) { return d * 0.017453292519943295f; }
float radToDeg(float r) { return r * 57.29577951308232f; }

float greatCircleKm(float lat1, float lon1, float lat2, float lon2) {
  constexpr float R = 6371.0f;
  float p1 = degToRad(lat1);
  float p2 = degToRad(lat2);
  float dp = degToRad(lat2 - lat1);
  float dl = degToRad(lon2 - lon1);

  float a = sinf(dp / 2.0f) * sinf(dp / 2.0f) +
            cosf(p1) * cosf(p2) * sinf(dl / 2.0f) * sinf(dl / 2.0f);
  float c = 2.0f * atan2f(sqrtf(a), sqrtf(1.0f - a));
  return R * c;
}

float initialBearingDeg(float lat1, float lon1, float lat2, float lon2) {
  float p1 = degToRad(lat1);
  float p2 = degToRad(lat2);
  float dl = degToRad(lon2 - lon1);

  float y = sinf(dl) * cosf(p2);
  float x = cosf(p1) * sinf(p2) - sinf(p1) * cosf(p2) * cosf(dl);
  float b = radToDeg(atan2f(y, x));
  if (b < 0.0f) b += 360.0f;
  return b;
}

float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

float mpsToKnots(float mps) {
  return mps * 1.9438445f;
}

int metersToFlightLevel(float meters) {
  float feet = meters * 3.28084f;
  if (feet < 0.0f) feet = 0.0f;
  return (int)(feet / 100.0f + 0.5f);
}

bool isInterestingIcaoType(const String& icaoTypeIn) {
  String t = icaoTypeIn;
  t.toUpperCase();
  t.trim();
  if (t.isEmpty()) return false;

  const char* exactTypes[] = {
    // Airbus
    "A20N", "A21N", "A318", "A319", "A320", "A321",
    "A332", "A333", "A338", "A339", "A342", "A343", "A345", "A346", 
    "A359", "A35K", "A388", "BCS1", "BCS3", "A306", "A310",
    // Boeing
    "B733", "B734", "B735", "B736", "B737", "B738", "B739", "B38M", "B39M", 
    "B744", "B748", "B74F", "B752", "B753", "B762", "B763", "B764", 
    "B772", "B773", "B77W", "B77L", "B77F", "B788", "B789", "B78X",
    // Regional and Private (Embraer, Bombardier, ATR, Dash 8, Pilatus, etc)
    "E170", "E175", "E190", "E195", "E290", "E295",
    "CRJ1", "CRJ2", "CRJ7", "CRJ9", "CRJX", "AT43", "AT45", "AT72", "AT75", "AT76", "DH8A", "DH8B", "DH8C", "DH8D", "PC24", 
    // Other Heavy / Cargo
    "MD11", "DC10", "MD1F", "DC1F", "L101", "A124", "A225", "IL76",
    // Military Heavy (Transport, AWACS, Tanker, Bomber)
    "C17", "C5", "C5M", "A400", "C130", "C30J", "K35R", "KC46", "KC10", "E3TF", "E3", "E4", "E6", "E8", "R135", "P8", "E767", "U2", "B1", "B2", "B52",
    // Military Fighters / Attack / VTOL
    "F15", "F16", "F18", "F22", "F35", "F117", "A10", "RFAL", "EUFI", "JAS3", "TNDO", "SU27", "SU30", "SU34", "SU35", "SU57", "MG29", "MG31", "MG35", "M346", "V22"
  };

  for (size_t i = 0; i < sizeof(exactTypes) / sizeof(exactTypes[0]); i++) {
    if (t == exactTypes[i]) return true;
  }

  return false;
}

int findAdsbCacheIndex(const String& modeS) {
  for (int i = 0; i < MAX_ADSB_CACHE; i++) {
    if (adsbCache[i].used && adsbCache[i].modeS == modeS) return i;
  }
  return -1;
}

int reserveAdsbCacheIndex() {
  for (int i = 0; i < MAX_ADSB_CACHE; i++) {
    if (!adsbCache[i].used) return i;
  }
  int oldest = 0;
  uint32_t maxAge = 0;
  uint32_t now = millis();
  for (int i = 0; i < MAX_ADSB_CACHE; i++) {
    uint32_t age = now - adsbCache[i].lastSeenMs;
    if (age >= maxAge) {
      maxAge = age;
      oldest = i;
    }
  }
  return oldest;
}

bool fetchAdsbdbRoute(const String& modeS, const String& callsign, String& outRouteIata) {
  if (WiFi.status() != WL_CONNECTED) return false;
  String cs = callsign;
  cs.trim();
  cs.replace(" ", "");
  if (cs == "UNK") return true;
  if (cs.isEmpty()) return true;

  WiFiClientSecure client;
  client.setInsecure();

  HTTPClient http;
  http.setConnectTimeout(2500);
  http.setTimeout(3500);

  String orig = "";
  String dest = "";
  String comboUrl = "https://api.adsbdb.com/v0/aircraft/" + modeS + "?callsign=" + urlEncode(cs);
  if (verboseLogging) debugLog("ADSBDB GET " + comboUrl);
  if (!http.begin(client, comboUrl)) {
    if (verboseLogging) debugLog("ADSBDB begin failed for " + modeS + " " + cs);
    return false;
  }

  int code = http.GET();
  if (verboseLogging) debugLog("ADSBDB HTTP " + String(code));
  String payload = http.getString();
  if (verboseLogging) {
    Serial.println("[VRB] ADSBDB payload:");
    Serial.println(payload);
  }
  http.end();

  if (code != HTTP_CODE_OK) return false;

  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, payload);
  if (err) {
    if (verboseLogging) debugLog("ADSBDB JSON err=" + String(err.c_str()));
    return false;
  }

  JsonVariant response = doc["response"];
  if (!response.isNull()) {
    JsonVariant fr = response["flightroute"];
    if (!fr.isNull()) {
      orig = fr["origin"]["iata_code"].isNull() ? "" : String((const char*)fr["origin"]["iata_code"]);
      dest = fr["destination"]["iata_code"].isNull() ? "" : String((const char*)fr["destination"]["iata_code"]);
    }
  }

  orig.toUpperCase();
  dest.toUpperCase();
  orig.trim();
  dest.trim();

  String route = "";
  if (!orig.isEmpty() && !dest.isEmpty()) route = orig + "-" + dest;
  else if (!orig.isEmpty()) route = orig + "-?";
  else if (!dest.isEmpty()) route = "?-" + dest;

  outRouteIata = route;
  return true;
}

void lookupOnePendingAdsbdbRoute() {
  if (WiFi.status() != WL_CONNECTED) return;
  if (aircraftCount <= 0) return;

  for (int i = 0; i < aircraftCount; i++) {
    String modeS = aircraft[i].modeS;
    modeS.trim();
    if (modeS.isEmpty()) continue;

    int idx = findAdsbCacheIndex(modeS);
    if (idx >= 0 && adsbCache[idx].used && adsbCache[idx].lookedUp) {
      if (adsbCache[idx].hasRoute && aircraft[i].routeIata != adsbCache[idx].routeIata) {
        aircraft[i].routeIata = adsbCache[idx].routeIata;
        dynamicDirty = true;
      }
      continue;
    }

    AdsbCacheEntry* cache = getOrAllocateCacheEntry(modeS);

    String routeIata = "";
    bool requestOk = fetchAdsbdbRoute(modeS, aircraft[i].callsign, routeIata);

    cache->lookedUp = true;
    cache->lastLookupMs = millis();
    cache->routeIata = routeIata;
    cache->hasRoute = requestOk && !routeIata.isEmpty();

    if (cache->hasRoute) {
      aircraft[i].routeIata = routeIata;
      dynamicDirty = true;
    }

    return;
  }
}

void applyCachedRoutesToCurrentAircraft() {
  for (int i = 0; i < aircraftCount; i++) {
    int idx = findAdsbCacheIndex(aircraft[i].modeS);
    if (idx < 0) continue;
    if (!adsbCache[idx].used || !adsbCache[idx].hasRoute) continue;
    aircraft[i].routeIata = adsbCache[idx].routeIata;
  }
}

bool getCachedRoute(const String& modeS, String& outRouteIata) {
  int idx = findAdsbCacheIndex(modeS);
  if (idx < 0) return false;
  if (!adsbCache[idx].used || !adsbCache[idx].hasRoute) return false;
  outRouteIata = adsbCache[idx].routeIata;
  return true;
}

String urlEncode(const String& in) {
  String out;
  out.reserve(in.length() * 3);
  const char* hex = "0123456789ABCDEF";

  for (size_t i = 0; i < in.length(); i++) {
    uint8_t c = (uint8_t)in[i];
    bool safe = (c >= 'a' && c <= 'z') ||
                (c >= 'A' && c <= 'Z') ||
                (c >= '0' && c <= '9') ||
                c == '-' || c == '_' || c == '.' || c == '~';
    if (safe) {
      out += (char)c;
    } else {
      out += '%';
      out += hex[(c >> 4) & 0x0F];
      out += hex[c & 0x0F];
    }
  }

  return out;
}

void saveConfig() {
  prefs.begin("radar", false);
  prefs.putString("ssid", cfg.ssid);
  prefs.putString("pass", cfg.pass);
  prefs.putFloat("clat", cfg.centerLat);
  prefs.putFloat("clon", cfg.centerLon);
  prefs.putFloat("rad", cfg.radiusKm);
  prefs.putFloat("spd", cfg.speedCutoffKts);
  prefs.putBool("showapt", cfg.showAirports);
  prefs.putString("cid", cfg.apiClientId);
  prefs.putString("csec", cfg.apiClientSecret);
  prefs.end();
}

void loadConfig() {
  prefs.begin("radar", true);
  cfg.ssid = prefs.getString("ssid", "");
  cfg.pass = prefs.getString("pass", "");
  cfg.centerLat = prefs.getFloat("clat", 48.8566f); // default: Paris
  cfg.centerLon = prefs.getFloat("clon", 2.3522f);
  cfg.radiusKm = prefs.getFloat("rad", 40.0f);
  cfg.speedCutoffKts = prefs.getFloat("spd", 200.0f);
  cfg.showAirports = prefs.getBool("showapt", true);
  cfg.apiClientId = prefs.getString("cid", "");
  cfg.apiClientSecret = prefs.getString("csec", "");
  prefs.end();

  cfg.radiusKm = clampf(cfg.radiusKm, 5.0f, 150.0f);
  cfg.speedCutoffKts = clampf(cfg.speedCutoffKts, 0.0f, 700.0f);
}

void saveAirportsToFlash() {
  if (cachedAirportCount <= 0) return;

  AirportCache cache;
  cache.count = cachedAirportCount;
  cache.savedLat = lastAptLat;
  cache.savedLon = lastAptLon;
  cache.savedRad = lastAptRad;
  memcpy(cache.airports, cachedAirports, sizeof(DynamicAirport) * cachedAirportCount);

  prefs.begin("radar", false);
  size_t written = prefs.putBytes("aptcache", &cache, sizeof(cache));
  prefs.end();

  if (written == sizeof(cache)) {
    debugLog("Airport cache saved to flash.");
  } else {
    debugLog("Failed to save airport cache to flash.");
  }
}

void loadAirportsFromFlash() {
  prefs.begin("radar", true);
  if (!prefs.isKey("aptcache")) {
    prefs.end();
    debugLog("No airport cache found in flash.");
    return;
  }

  AirportCache cache;
  size_t read = prefs.getBytes("aptcache", &cache, sizeof(cache));
  prefs.end();

  if (read != sizeof(cache)) {
    debugLog("Airport cache in flash is corrupt/wrong size.");
    return;
  }

  // Check if the cache is for the current area
  float latDiff = fabsf(cache.savedLat - cfg.centerLat);
  float lonDiff = fabsf(cache.savedLon - cfg.centerLon);
  float radDiff = fabsf(cache.savedRad - cfg.radiusKm);

  if (latDiff < 0.01f && lonDiff < 0.01f && radDiff < 1.0f) {
    cachedAirportCount = cache.count;
    if (cachedAirportCount > MAX_AIRPORTS) cachedAirportCount = MAX_AIRPORTS; // sanity check
    if (cachedAirportCount > 0) {
      memcpy(cachedAirports, cache.airports, sizeof(DynamicAirport) * cachedAirportCount);
    }
    
    lastAptLat = cache.savedLat;
    lastAptLon = cache.savedLon;
    lastAptRad = cache.savedRad;

    debugLog("Loaded " + String(cachedAirportCount) + " airports from flash cache.");
    dynamicDirty = true;
  } else {
    debugLog("Airport cache in flash is for a different area. Ignoring.");
  }
}

void publishStatus(const String& text) {
  lastStatusText = text;
  if (statusChar) {
    statusChar->setValue(text.c_str());
    statusChar->notify();
  }
  Serial.println("[STATUS] " + text);
  dynamicDirty = true;
}

void connectWiFi() {
  if (cfg.ssid.isEmpty()) {
    wifiConnected = false;
    publishStatus("WiFi SSID missing");
    return;
  }

  debugLog("WiFi begin for SSID=" + cfg.ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(cfg.ssid.c_str(), cfg.pass.c_str());

  publishStatus("WiFi connecting...");
  uint32_t start = millis();
  wl_status_t prev = WiFi.status();
  debugLog("WiFi status=" + wifiStatusToText(prev));
  while (WiFi.status() != WL_CONNECTED && millis() - start < 12000) {
    wl_status_t now = WiFi.status();
    if (now != prev) {
      prev = now;
      debugLog("WiFi status=" + wifiStatusToText(now));
    }
    delay(250);
  }

  wifiConnected = WiFi.status() == WL_CONNECTED;
  if (wifiConnected) {
    debugLog("WiFi RSSI=" + String(WiFi.RSSI()) + " dBm");
    publishStatus("WiFi OK " + WiFi.localIP().toString());
  } else {
    debugLog("WiFi final status=" + wifiStatusToText(WiFi.status()));
    publishStatus("WiFi failed");
  }
}

void checkForUpdates() {
  if (WiFi.status() != WL_CONNECTED) return;

  tft.fillScreen(0x0000);
  tft.setTextColor(0xFFFF);
  tft.setTextSize(2);
  tft.setCursor(96, CENTER_Y - 20);
  tft.print("v" + String(CURRENT_VERSION, 1));
  tft.setCursor(54, CENTER_Y + 10);
  tft.print("Checking...");

  publishStatus("Checking updates...");
  debugLog("Checking for updates at: " + String(FW_VERSION_URL));

  WiFiClientSecure client;
  client.setInsecure(); // GitHub raw requires HTTPS

  HTTPClient http;
  http.setConnectTimeout(5000);
  http.setTimeout(5000);

  // Step 1: Check version
  if (!http.begin(client, FW_VERSION_URL)) {
    debugLog("Update: HTTP begin failed");
    return;
  }

  int code = http.GET();
  if (code != HTTP_CODE_OK) {
    debugLog("Update: version check failed, HTTP " + String(code));
    http.end();
    return;
  }

  String payload = http.getString();
  http.end();

  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, payload);
  if (err) {
    debugLog("Update: JSON parse failed");
    return;
  }
  
  // Grab the version, fallback to CURRENT_VERSION if the JSON is malformed
  float availableVersion = doc["version"] | CURRENT_VERSION;

  debugLog("Current version: " + String(CURRENT_VERSION) + ", Available: " + String(availableVersion));

  // Step 2: Download and apply update if needed
  if (availableVersion > CURRENT_VERSION) {
    tft.fillScreen(0x0000);
    tft.setCursor(66, CENTER_Y - 20);
    tft.print("New: v" + String(availableVersion, 1));
    tft.setCursor(54, CENTER_Y + 10);
    tft.print("Updating...");

    publishStatus("Updating fw...");
    debugLog("New version found! Starting OTA update from: " + String(FW_BIN_URL));
    
    // This will stream the .bin directly to flash and automatically reboot if successful.
    t_httpUpdate_return ret = httpUpdate.update(client, FW_BIN_URL);

    tft.fillScreen(0x0000);
    tft.setCursor(42, CENTER_Y);
    switch (ret) {
      case HTTP_UPDATE_FAILED:
        debugLog("Update failed: " + httpUpdate.getLastErrorString());
        publishStatus("OTA Failed");
        tft.print("Update Failed");
        delay(2000);
        break;
      case HTTP_UPDATE_NO_UPDATES:
        debugLog("Update no updates.");
        break;
      case HTTP_UPDATE_OK:
        // Normally won't be reached because of auto-reboot
        debugLog("Update OK."); 
        break;
    }
  } else {
    tft.fillScreen(0x0000);
    tft.setCursor(54, CENTER_Y);
    tft.print("Up to date!");
    debugLog("Firmware is up to date.");
    delay(1000);
  }
}

class RadarServerCallbacks : public NimBLEServerCallbacks {
 public:
  void onConnect(NimBLEServer* s) override {
    (void)s;
    bleClientConnected = true;
    debugLog("BLE client connected");
    publishStatus("BLE connected");
  }

  void onDisconnect(NimBLEServer* s) override {
    (void)s;
    bleClientConnected = false;
    debugLog("BLE client disconnected");
    publishStatus("BLE disconnected");
    NimBLEDevice::startAdvertising();
  }
};

class GenericWriteCallback : public NimBLECharacteristicCallbacks {
 public:
  explicit GenericWriteCallback(const String& key) : key_(key) {}

 protected:
  void onWrite(NimBLECharacteristic* c) override {
    String v = String(c->getValue().c_str());
    v.trim();
    debugLog("BLE write key=" + key_ + " value='" + v + "'");

    if (key_ == "ssid") cfg.ssid = v;
    else if (key_ == "pass") cfg.pass = v;
    else if (key_ == "lat") cfg.centerLat = clampf(v.toFloat(), -85.0f, 85.0f);
    else if (key_ == "lon") cfg.centerLon = clampf(v.toFloat(), -180.0f, 180.0f);
    else if (key_ == "radius") cfg.radiusKm = clampf(v.toFloat(), 5.0f, 150.0f);
    else if (key_ == "speed") cfg.speedCutoffKts = clampf(v.toFloat(), 0.0f, 700.0f);
    else if (key_ == "client_id") {
      cfg.apiClientId = v;
      saveConfig();
      publishStatus("client_id saved");
    }
    else if (key_ == "client_secret") {
      cfg.apiClientSecret = v;
      saveConfig();
      publishStatus("client_secret saved");
    }
    else if (key_ == "cmd") {
      String cmd = v;
      cmd.toLowerCase();

      if (cmd == "save") {
        saveConfig();
        printConfigSummary();
        publishStatus("Config saved");
      } else if (cmd == "apply") {
        saveConfig();
        printConfigSummary();
        connectWiFi();
      } else if (cmd == "auth") {
        publishStatus("Auth not needed for ADSB.fi");
      } else if (cmd == "verbose" || cmd == "verbose on") {
        verboseLogging = true;
        publishStatus("Verbose ON");
      } else if (cmd == "verbose off") {
        verboseLogging = false;
        publishStatus("Verbose OFF");
      } else if (cmd == "airports on") {
        cfg.showAirports = true;
        publishStatus("Airports ON");
      } else if (cmd == "airports off") {
        cfg.showAirports = false;
        publishStatus("Airports OFF");
      } else if (cmd == "clearwifi") {
        cfg.ssid = "";
        cfg.pass = "";
        saveConfig();
        WiFi.disconnect(true, true);
        wifiConnected = false;
        publishStatus("WiFi creds cleared");
      } else if (cmd == "reboot") {
        publishStatus("Rebooting...");
        delay(300);
        ESP.restart();
      } else {
        publishStatus("Unknown cmd");
      }
    }
  }

 private:
  String key_;
};

void setupBLE() {
  uint64_t chipid = ESP.getEfuseMac();
  char nameBuf[32];
  snprintf(nameBuf, sizeof(nameBuf), "DeskRadar-%04X", (uint16_t)(chipid & 0xFFFF));

  NimBLEDevice::init(nameBuf);
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);

  NimBLEServer* server = NimBLEDevice::createServer();
  server->setCallbacks(new RadarServerCallbacks());
  NimBLEService* service = server->createService(RADAR_SERVICE_UUID);

  auto* ssidChar = service->createCharacteristic(SSID_CHAR_UUID, NIMBLE_PROPERTY::WRITE);
  auto* passChar = service->createCharacteristic(PASS_CHAR_UUID, NIMBLE_PROPERTY::WRITE);
  auto* latChar = service->createCharacteristic(LAT_CHAR_UUID, NIMBLE_PROPERTY::WRITE);
  auto* lonChar = service->createCharacteristic(LON_CHAR_UUID, NIMBLE_PROPERTY::WRITE);
  auto* radiusChar = service->createCharacteristic(RADIUS_CHAR_UUID, NIMBLE_PROPERTY::WRITE);
  auto* speedChar = service->createCharacteristic(SPEED_CHAR_UUID, NIMBLE_PROPERTY::WRITE);
  auto* clientIdChar = service->createCharacteristic(CLIENT_ID_CHAR_UUID, NIMBLE_PROPERTY::WRITE);
  auto* clientSecretChar = service->createCharacteristic(CLIENT_SECRET_CHAR_UUID, NIMBLE_PROPERTY::WRITE);
  auto* cmdChar = service->createCharacteristic(CMD_CHAR_UUID, NIMBLE_PROPERTY::WRITE);
  statusChar = service->createCharacteristic(STATUS_CHAR_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

  ssidChar->setCallbacks(new GenericWriteCallback("ssid"));
  passChar->setCallbacks(new GenericWriteCallback("pass"));
  latChar->setCallbacks(new GenericWriteCallback("lat"));
  lonChar->setCallbacks(new GenericWriteCallback("lon"));
  radiusChar->setCallbacks(new GenericWriteCallback("radius"));
  speedChar->setCallbacks(new GenericWriteCallback("speed"));
  clientIdChar->setCallbacks(new GenericWriteCallback("client_id"));
  clientSecretChar->setCallbacks(new GenericWriteCallback("client_secret"));
  cmdChar->setCallbacks(new GenericWriteCallback("cmd"));

  statusChar->setValue("Ready");

  service->start();

  NimBLEAdvertising* advertising = NimBLEDevice::getAdvertising();
  advertising->addServiceUUID(RADAR_SERVICE_UUID);
  advertising->setScanResponse(true);
  advertising->start();

  debugLog("BLE ready, advertising as " + String(nameBuf));
}

bool fetchAdsbFi() {
  uint32_t nowMs = millis();
  fetchEpoch++;

  if (WiFi.status() != WL_CONNECTED) {
    wifiConnected = false;

    // Try to recover automatically after connection loss / startup failure.
    if (nowMs - lastWiFiRetryMs > 30000UL) {
      lastWiFiRetryMs = nowMs;
      debugLog("WiFi retry triggered, status=" + wifiStatusToText(WiFi.status()));
      connectWiFi();
    }

    if (WiFi.status() != WL_CONNECTED) {
      if (nowMs - lastNoWiFiStatusMs > 15000UL) {
        lastNoWiFiStatusMs = nowMs;
        debugLog("ADSB.fi skipped, WiFi not connected (" + wifiStatusToText(WiFi.status()) + ")");
        publishStatus("No WiFi for API");
      }
      return false;
    }
  }

  if (WiFi.status() == WL_CONNECTED) {
    wifiConnected = true;
    lastNoWiFiStatusMs = 0;
  } else {
    return false;
  }

  // Calculate distance in nautical miles (Max 250NM for adsb.fi API)
  float distNm = cfg.radiusKm / 1.852f;
  if (distNm > 250.0f) distNm = 250.0f;

  String url = "https://opendata.adsb.fi/api/v3/lat/" + String(cfg.centerLat, 5) +
               "/lon/" + String(cfg.centerLon, 5) +
               "/dist/" + String(distNm, 1);
               
  debugLog("ADSB.fi GET " + url);

  WiFiClientSecure client;
  client.setInsecure();

  HTTPClient http;
  http.setConnectTimeout(7000);
  http.setTimeout(9000);
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);

  if (!http.begin(client, url)) {
    publishStatus("HTTP begin failed");
    return false;
  }

  int code = http.GET();
  lastHttpCode = code;
  
  if (code != HTTP_CODE_OK) {
    publishStatus("ADSB.fi HTTP " + String(code));
    http.end();
    return false;
  }

  String payload = http.getString();
  debugLog("ADSB.fi payload bytes=" + String(payload.length()));
  if (verboseLogging) {
    Serial.println("[VRB] ADSB.fi response payload:");
    Serial.println(payload);
  }

#if ARDUINOJSON_VERSION_MAJOR == 6
  DynamicJsonDocument doc(32768);
#else
  JsonDocument doc;
#endif
  DeserializationError err = deserializeJson(doc, payload);
  http.end();

  if (err) {
    String preview = payload.substring(0, 120);
    preview.replace("\n", " ");
    preview.replace("\r", " ");
    debugLog("JSON err=" + String(err.c_str()) + " preview='" + preview + "'");
    publishStatus("JSON err: " + String(err.c_str()));
    return false;
  }

  // Extract the aircraft array (commonly "aircraft", occasionally "ac" in derivatives)
  JsonArray aircraftList = doc["aircraft"].as<JsonArray>();
  if (aircraftList.isNull()) {
    aircraftList = doc["ac"].as<JsonArray>(); 
  }

  if (aircraftList.isNull()) {
    aircraftCount = 0;
    return true;
  }

  int count = 0;
  for (JsonVariant v : aircraftList) {
    if (count >= MAX_AIRCRAFT) break;

    // Reject incomplete position data
    if (v["lat"].isNull() || v["lon"].isNull()) continue;

    float lat = v["lat"].as<float>();
    float lon = v["lon"].as<float>();
    
    // Ensure strict radar boundary check
    float dist = greatCircleKm(cfg.centerLat, cfg.centerLon, lat, lon);
    if (dist > cfg.radiusKm) continue;

    String callsign = v["flight"].isNull() ? "" : String((const char*)v["flight"]);
    callsign.trim();
    if (callsign.isEmpty()) callsign = "UNK";

    String modeS = v["hex"].isNull() ? "" : String((const char*)v["hex"]);
    modeS.toUpperCase();
    modeS.trim();
    if (modeS.isEmpty()) continue;

    float trackDeg = v["track"].isNull() ? 0.0f : v["track"].as<float>();
    
    // speed is reported in knots by adsb.fi
    float gsKts = v["gs"].isNull() ? 0.0f : v["gs"].as<float>();
    
    // Convert to meters/sec to match the legacy internal math (for drawing/logic)
    float velocityMps = gsKts * 0.514444f;

    // altitude is reported in feet (either geometric or barometric) by adsb.fi
    float altFeet = 0.0f;
    if (v["alt_geom"].is<float>() || v["alt_geom"].is<int>()) {
      altFeet = v["alt_geom"].as<float>();
    } else if (v["alt_baro"].is<float>() || v["alt_baro"].is<int>()) {
      altFeet = v["alt_baro"].as<float>();
    }
    
    // Convert to meters to match the legacy internal math
    float altitudeM = altFeet * 0.3048f;

    aircraft[count].callsign = callsign;
    aircraft[count].modeS = modeS;
    
    // If the DB includes 't' for type, map it
    String icaoType = v["t"].isNull() ? "" : String((const char*)v["t"]);
    aircraft[count].icaoType = icaoType;

    if (!isInterestingIcaoType(icaoType)) continue;
    
    aircraft[count].routeIata = "";
    String cachedRoute;
    if (getCachedRoute(modeS, cachedRoute)) aircraft[count].routeIata = cachedRoute;
    
    aircraft[count].lat = lat;
    aircraft[count].lon = lon;
    aircraft[count].trackDeg = trackDeg;
    aircraft[count].velocity = velocityMps;
    aircraft[count].altitudeM = altitudeM;

    count++;
  }

  aircraftCount = count;
  applyCachedRoutesToCurrentAircraft();
  successfulFetchCount++;
  debugLog("ADSB.fi parsed aircraft=" + String(aircraftCount));
  publishStatus("Aircraft: " + String(aircraftCount));
  dynamicDirty = true;
  return true;
}

uint16_t radarGreen(uint8_t intensity) {
  return ((uint16_t)(intensity & 0xFC) << 3);
}

void drawGrid(Adafruit_GFX& gfx) {
  uint16_t grid = radarGreen(65);
  gfx.drawCircle(CENTER_X, CENTER_Y, RADAR_RADIUS_PX, grid);
  gfx.drawCircle(CENTER_X, CENTER_Y, RADAR_RADIUS_PX * 3 / 4, grid);
  gfx.drawCircle(CENTER_X, CENTER_Y, RADAR_RADIUS_PX / 2, grid);
  gfx.drawCircle(CENTER_X, CENTER_Y, RADAR_RADIUS_PX / 4, grid);

  gfx.drawLine(CENTER_X - RADAR_RADIUS_PX, CENTER_Y, CENTER_X + RADAR_RADIUS_PX, CENTER_Y, grid);
  gfx.drawLine(CENTER_X, CENTER_Y - RADAR_RADIUS_PX, CENTER_X, CENTER_Y + RADAR_RADIUS_PX, grid);

  for (int d = 45; d < 360; d += 90) {
    float a = degToRad((float)d);
    int x = CENTER_X + (int)(sinf(a) * RADAR_RADIUS_PX);
    int y = CENTER_Y - (int)(cosf(a) * RADAR_RADIUS_PX);
    gfx.drawLine(CENTER_X, CENTER_Y, x, y, radarGreen(35));
  }
}

bool pointInRadarCircle(int x, int y) {
  int dx = x - CENTER_X;
  int dy = y - CENTER_Y;
  return (dx * dx + dy * dy) <= (RADAR_RADIUS_PX * RADAR_RADIUS_PX);
}

bool updateAirportsIfMoved() {
  if (!cfg.showAirports) return false;
  if (WiFi.status() != WL_CONNECTED) return false;

  // Check if we moved enough to warrant an update (1km distance or radius change)
  float distMoved = greatCircleKm(lastAptLat, lastAptLon, cfg.centerLat, cfg.centerLon);
  if (distMoved < 1.0f && fabsf(lastAptRad - cfg.radiusKm) < 1.0f) {
    return false;
  }

  // Use OpenStreetMap Overpass API - 100% free, no API key needed
  String query = "[out:json];nwr[\"aeroway\"=\"aerodrome\"][\"iata\"](around:" + 
                 String(cfg.radiusKm * 1000.0f, 0) + "," + 
                 String(cfg.centerLat, 5) + "," + String(cfg.centerLon, 5) + ");out center;";
  String url = "https://overpass-api.de/api/interpreter?data=" + urlEncode(query);

  debugLog("Overpass API GET " + url);

  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;
  http.setConnectTimeout(4000);
  http.setTimeout(6000); // Overpass can sometimes take a moment
  
  if (!http.begin(client, url)) return false;
  http.addHeader("User-Agent", "DeskRadar-ESP32");

  int code = http.GET();
  if (code != HTTP_CODE_OK) {
    debugLog("Overpass API HTTP " + String(code));
    http.end();
    return false;
  }

  String payload = http.getString();
  http.end();

#if ARDUINOJSON_VERSION_MAJOR == 6
  DynamicJsonDocument doc(8192);
#else
  JsonDocument doc;
#endif
  DeserializationError err = deserializeJson(doc, payload);
  if (err) {
    debugLog("Airports JSON err: " + String(err.c_str()));
    return false;
  }

  JsonArray arr = doc["elements"].as<JsonArray>();
  if (arr.isNull()) return false;

  cachedAirportCount = 0;
  for (JsonVariant v : arr) {
    if (cachedAirportCount >= MAX_AIRPORTS) break;
    
    String iataStr = v["tags"]["iata"].isNull() ? "" : v["tags"]["iata"].as<String>();
    if (iataStr.isEmpty() || iataStr.length() > 4) continue; // Skip small airstrips or invalid IATA
    
    memset(cachedAirports[cachedAirportCount].iata, 0, 5);
    strncpy(cachedAirports[cachedAirportCount].iata, iataStr.c_str(), 4);
    // Nodes have "lat"/"lon", Ways/Relations have "center"
    cachedAirports[cachedAirportCount].lat = v["lat"].isNull() ? v["center"]["lat"].as<float>() : v["lat"].as<float>();
    cachedAirports[cachedAirportCount].lon = v["lon"].isNull() ? v["center"]["lon"].as<float>() : v["lon"].as<float>();
    cachedAirportCount++;
  }

  lastAptLat = cfg.centerLat;
  lastAptLon = cfg.centerLon;
  lastAptRad = cfg.radiusKm;
  debugLog("Updated local airports cache: " + String(cachedAirportCount));
  saveAirportsToFlash();
  return true;
}

void drawAirports(Adafruit_GFX& gfx) {
  if (!cfg.showAirports) return;

  for (int i = 0; i < cachedAirportCount; i++) {
    float dist = greatCircleKm(cfg.centerLat, cfg.centerLon, cachedAirports[i].lat, cachedAirports[i].lon);
    if (dist > cfg.radiusKm) continue;

    float bearing = initialBearingDeg(cfg.centerLat, cfg.centerLon, cachedAirports[i].lat, cachedAirports[i].lon);
    float r = (dist / cfg.radiusKm) * RADAR_RADIUS_PX;
    float ang = degToRad(bearing);
    int x = CENTER_X + (int)(sinf(ang) * r);
    int y = CENTER_Y - (int)(cosf(ang) * r);

    if (!pointInRadarCircle(x, y)) continue;

    uint16_t color = radarGreen(120);
    gfx.drawLine(x - 3, y, x + 3, y, color);
    gfx.drawLine(x, y - 3, x, y + 3, color);
    
    gfx.setTextColor(color);
    gfx.setTextSize(1);
    gfx.setCursor(x + 4, y - 4);
    gfx.print(cachedAirports[i].iata);
  }
}

bool rectInsideRadarCircle(int x, int y, int w, int h) {
  if (x < 0 || y < 0 || x + w > SCREEN_W || y + h > SCREEN_H) return false;
  return pointInRadarCircle(x, y) &&
         pointInRadarCircle(x + w - 1, y) &&
         pointInRadarCircle(x, y + h - 1) &&
         pointInRadarCircle(x + w - 1, y + h - 1);
}

bool rectsOverlap(const RectRegion& a, const RectRegion& b) {
  return !(a.x + a.w <= b.x || b.x + b.w <= a.x || a.y + a.h <= b.y || b.y + b.h <= a.y);
}

void pushCanvasRegion(int x, int y, int w, int h) {
  int x0 = max(0, x);
  int y0 = max(0, y);
  int x1 = min(SCREEN_W, x + w);
  int y1 = min(SCREEN_H, y + h);
  int rw = x1 - x0;
  int rh = y1 - y0;
  if (rw <= 0 || rh <= 0) return;

  uint16_t* buf = (uint16_t*)malloc(rw * rh * 2);
  uint16_t* src = canvas.getBuffer();

  if (buf) {
    for (int yy = 0; yy < rh; yy++) {
      memcpy(&buf[yy * rw], &src[(y0 + yy) * SCREEN_W + x0], rw * 2);
    }
    tft.drawRGBBitmap(x0, y0, buf, rw, rh);
    free(buf);
  } else {
    // Fallback to pushing line-by-line if malloc fails
    for (int yy = 0; yy < rh; yy++) {
      tft.drawRGBBitmap(x0, y0 + yy, &src[(y0 + yy) * SCREEN_W + x0], rw, 1);
    }
  }
}

bool rectOverlapsAircraftMarker(const RectRegion& r, int markerX, int markerY) {
  RectRegion markerBox = {
    (int16_t)(markerX - 5),
    (int16_t)(markerY - 5),
    11,
    11
  };
  return rectsOverlap(r, markerBox);
}

AdsbCacheEntry* getOrAllocateCacheEntry(const String& modeS) {
  int idx = findAdsbCacheIndex(modeS);
  if (idx < 0) {
    idx = reserveAdsbCacheIndex();
    adsbCache[idx].used = true;
    adsbCache[idx].modeS = modeS;
    adsbCache[idx].routeIata = "";
    adsbCache[idx].lookedUp = false;
    adsbCache[idx].hasRoute = false;
    adsbCache[idx].hasLabelAngle = false;
    adsbCache[idx].color = planeColors[nextColorIdx];
    nextColorIdx = (nextColorIdx + 1) % NUM_COLORS;
    adsbCache[idx].trailHead = 0;
    adsbCache[idx].trailCount = 0;
    adsbCache[idx].lastLookupMs = 0;
  }
  adsbCache[idx].lastSeenMs = millis();
  return &adsbCache[idx];
}

RectRegion getRectForAngle(float angle, int markerX, int markerY, int tagW, int tagH) {
  int dist = 16;
  int cx = markerX + (int)(cosf(angle) * dist);
  int cy = markerY + (int)(sinf(angle) * dist);
  int tx = cx;
  int ty = cy;
  if (cosf(angle) < 0) tx -= tagW;
  if (sinf(angle) < 0) ty -= tagH;
  return {(int16_t)tx, (int16_t)ty, (int16_t)tagW, (int16_t)tagH};
}

bool isTargetValid(const RectRegion& cand, int myIdx, const int markerXs[], const int markerYs[], const bool markerValid[], int markerCount, const RectRegion placedTargets[], int placedCount) {
  if (cand.x < 0 || cand.y < 0 || cand.x + cand.w > SCREEN_W || cand.y + cand.h > SCREEN_H) return false;
  for (int j = 0; j < placedCount; j++) {
    if (rectsOverlap(cand, placedTargets[j])) return false;
  }
  for (int j = 0; j < markerCount; j++) {
    if (!markerValid[j] || j == myIdx) continue;
    if (rectOverlapsAircraftMarker(cand, markerXs[j], markerYs[j])) return false;
  }
  return true;
}

void drawDashedLine(Adafruit_GFX& gfx, int x0, int y0, int x1, int y1, uint16_t color, int dashLen) {
  int dx = x1 - x0;
  int dy = y1 - y0;
  float dist = sqrtf(dx * dx + dy * dy);
  if (dist <= 0) return;
  int steps = (int)(dist / (dashLen * 2));
  if (steps == 0) {
    gfx.drawLine(x0, y0, x1, y1, color);
    return;
  }
  float xStep = (float)dx / (steps * 2);
  float yStep = (float)dy / (steps * 2);
  for (int i = 0; i < steps * 2; i += 2) {
    int startX = x0 + (int)(i * xStep);
    int startY = y0 + (int)(i * yStep);
    int endX = x0 + (int)((i + 1) * xStep);
    int endY = y0 + (int)((i + 1) * yStep);
    gfx.drawLine(startX, startY, endX, endY, color);
  }
}

void drawThickLine(Adafruit_GFX& gfx, int x0, int y0, int x1, int y1, uint16_t color) {
  gfx.drawLine(x0, y0, x1, y1, color);
  gfx.drawLine(x0 + 1, y0, x1 + 1, y1, color);
  gfx.drawLine(x0, y0 + 1, x1, y1 + 1, color);
}

struct TagToDraw {
  int tx;
  int ty;
  int tagW;
  int tagH;
  String cs;
  int fl;
  int kts;
  String route;
  uint16_t color;
};

void drawAircraft(Adafruit_GFX& gfx) {
  int markerX[MAX_AIRCRAFT];
  int markerY[MAX_AIRCRAFT];
  bool markerValid[MAX_AIRCRAFT];

  for (int i = 0; i < MAX_AIRCRAFT; i++) {
    markerX[i] = 0;
    markerY[i] = 0;
    markerValid[i] = false;
  }

  // Pass 1: compute and draw all markers/heading first.
  for (int i = 0; i < aircraftCount; i++) {
    float dist = greatCircleKm(cfg.centerLat, cfg.centerLon, aircraft[i].lat, aircraft[i].lon);
    float bearing = initialBearingDeg(cfg.centerLat, cfg.centerLon, aircraft[i].lat, aircraft[i].lon);

    float r = (dist / cfg.radiusKm) * RADAR_RADIUS_PX;
    if (r > RADAR_RADIUS_PX) continue;

    float ang = degToRad(bearing);
    int x = CENTER_X + (int)(sinf(ang) * r);
    int y = CENTER_Y - (int)(cosf(ang) * r);
    if (!pointInRadarCircle(x, y)) continue;

    markerX[i] = x;
    markerY[i] = y;
    markerValid[i] = true;

    AdsbCacheEntry* cache = getOrAllocateCacheEntry(aircraft[i].modeS);

    // Add to trail if moved
    if (cache->trailCount == 0) {
      cache->trail[0] = {aircraft[i].lat, aircraft[i].lon};
      cache->trailHead = 1;
      cache->trailCount = 1;
    } else {
      int lastIdx = (cache->trailHead - 1 + MAX_TRAIL) % MAX_TRAIL;
      float movedKm = greatCircleKm(cache->trail[lastIdx].lat, cache->trail[lastIdx].lon, aircraft[i].lat, aircraft[i].lon);
      float kmPerPixel = cfg.radiusKm / (float)RADAR_RADIUS_PX;
      
      if (movedKm > 20.0f) {
        // The plane jumped an enormous distance; reset the trail cache to prevent a line drawing completely across the screen
        cache->trail[0] = {aircraft[i].lat, aircraft[i].lon};
        cache->trailHead = 1;
        cache->trailCount = 1;
      } else if (movedKm > (kmPerPixel * 3.0f)) {
        // Add a new dot in the trail only once it has moved about 3 screen pixels
        cache->trail[cache->trailHead] = {aircraft[i].lat, aircraft[i].lon};
        cache->trailHead = (cache->trailHead + 1) % MAX_TRAIL;
        if (cache->trailCount < MAX_TRAIL) cache->trailCount++;
      }
    }

    // Draw trail
    if (cache->trailCount > 1) {
      int prevX = -1, prevY = -1;
      int minX = 9999, minY = 9999, maxX = -9999, maxY = -9999;
      for (int t = 0; t < cache->trailCount; t++) {
        int idx = (cache->trailHead - 1 - t + MAX_TRAIL) % MAX_TRAIL;
        float tLat = cache->trail[idx].lat;
        float tLon = cache->trail[idx].lon;
        float tDist = greatCircleKm(cfg.centerLat, cfg.centerLon, tLat, tLon);
        float tBear = initialBearingDeg(cfg.centerLat, cfg.centerLon, tLat, tLon);
        float tR = (tDist / cfg.radiusKm) * RADAR_RADIUS_PX;
        float tAng = degToRad(tBear);
        int px = CENTER_X + (int)(sinf(tAng) * tR);
        int py = CENTER_Y - (int)(cosf(tAng) * tR);
        
        if (prevX != -1 && prevY != -1) {
          // Only connect alternating points to build a very fast dashed line
          if (t % 2 == 1) {
            gfx.drawLine(prevX, prevY, px, py, cache->color);
          }
          minX = min(minX, min(prevX, px));
          minY = min(minY, min(prevY, py));
          maxX = max(maxX, max(prevX, px));
          maxY = max(maxY, max(prevY, py));
        }
        prevX = px;
        prevY = py;
      }
      if (minX != 9999) {
        addCurrentRegion(minX - 2, minY - 2, maxX - minX + 4, maxY - minY + 4);
      }
    }

    gfx.fillCircle(x, y, 2, cache->color);
    gfx.drawCircle(x, y, 4, cache->color);
    addCurrentRegion(x - 5, y - 5, 11, 11);

    float hdg = degToRad(aircraft[i].trackDeg);
    int hx = x + (int)(sinf(hdg) * 9.0f);
    int hy = y - (int)(cosf(hdg) * 9.0f);
    gfx.drawLine(x, y, hx, hy, cache->color);
    int lx = min(x, hx) - 1;
    int ly = min(y, hy) - 1;
    addCurrentRegion(lx, ly, abs(hx - x) + 3, abs(hy - y) + 3);
  }

  // Pass 2: calculate target angles and draw thick leader lines.
  RectRegion placedTargets[MAX_TAGS_ON_SCREEN];
  TagToDraw tagsToDraw[MAX_TAGS_ON_SCREEN];
  int tagsDrawn = 0;

  for (int i = 0; i < aircraftCount; i++) {
    if (!markerValid[i]) continue;
    if (tagsDrawn >= MAX_TAGS_ON_SCREEN) break;
    
    AdsbCacheEntry* cache = getOrAllocateCacheEntry(aircraft[i].modeS);
    if (!cache->hasLabelAngle) {
      cache->targetLabelAngle = -0.785398f; // Start everyone sliding out to Top-Right (-PI/4)
      cache->currentLabelAngle = -0.785398f;
      cache->hasLabelAngle = true;
    }

    int x = markerX[i];
    int y = markerY[i];

    String cs = aircraft[i].callsign;
    if (cs.isEmpty()) cs = "UNK";
    if (cs.length() > 8) cs = cs.substring(0, 8);

    int fl = metersToFlightLevel(aircraft[i].altitudeM);
    int kts = (int)(mpsToKnots(aircraft[i].velocity) + 0.5f);
    String route = aircraft[i].routeIata;
    route.trim();
    bool hasRoute = !route.isEmpty();

    // Dynamically calculate the widest line of text (Adafruit GFX default font is 6px per char)
    String line2 = "FL" + String(fl) + " " + String(kts);
    int maxChars = cs.length();
    if (line2.length() > maxChars) maxChars = line2.length();
    if (hasRoute && route.length() > maxChars) maxChars = route.length();
    
    int tagW = maxChars * 6;
    int tagH = hasRoute ? 27 : 18;

    // Find best target angle
    float bestAngle = cache->targetLabelAngle;
    RectRegion cand = getRectForAngle(bestAngle, x, y, tagW, tagH);
    if (!isTargetValid(cand, i, markerX, markerY, markerValid, aircraftCount, placedTargets, tagsDrawn)) {
      bool found = false;
      for (float delta = 0.2f; delta <= 3.15f; delta += 0.2f) {
        cand = getRectForAngle(cache->targetLabelAngle + delta, x, y, tagW, tagH);
        if (isTargetValid(cand, i, markerX, markerY, markerValid, aircraftCount, placedTargets, tagsDrawn)) {
          bestAngle = cache->targetLabelAngle + delta;
          found = true; break;
        }
        cand = getRectForAngle(cache->targetLabelAngle - delta, x, y, tagW, tagH);
        if (isTargetValid(cand, i, markerX, markerY, markerValid, aircraftCount, placedTargets, tagsDrawn)) {
          bestAngle = cache->targetLabelAngle - delta;
          found = true; break;
        }
      }
    }
    cache->targetLabelAngle = bestAngle;
    placedTargets[tagsDrawn] = getRectForAngle(bestAngle, x, y, tagW, tagH);

    // Slide current angle
    float diff = cache->targetLabelAngle - cache->currentLabelAngle;
    while (diff > PI) diff -= 2.0f * PI;
    while (diff <= -PI) diff += 2.0f * PI;
    float maxSpeed = 0.15f; // Slide speed limit (radians per frame)
    if (diff > maxSpeed) diff = maxSpeed;
    if (diff < -maxSpeed) diff = -maxSpeed;
    cache->currentLabelAngle += diff;
    while (cache->currentLabelAngle > PI) cache->currentLabelAngle -= 2.0f * PI;
    while (cache->currentLabelAngle <= -PI) cache->currentLabelAngle += 2.0f * PI;

    // Get actual drawing rect
    RectRegion curRect = getRectForAngle(cache->currentLabelAngle, x, y, tagW, tagH);

    // Save to draw tags later
    tagsToDraw[tagsDrawn].tx = curRect.x;
    tagsToDraw[tagsDrawn].ty = curRect.y;
    tagsToDraw[tagsDrawn].tagW = tagW;
    tagsToDraw[tagsDrawn].tagH = tagH;
    tagsToDraw[tagsDrawn].cs = cs;
    tagsToDraw[tagsDrawn].fl = fl;
    tagsToDraw[tagsDrawn].kts = kts;
    tagsToDraw[tagsDrawn].route = route;
    tagsToDraw[tagsDrawn].color = cache->color;

    tagsDrawn++;
  }

  // Pass 3: draw labels with transparent background
  for (int j = 0; j < tagsDrawn; j++) {
    TagToDraw& t = tagsToDraw[j];

    gfx.setTextColor(t.color);
    gfx.setCursor(t.tx, t.ty);
    gfx.print(t.cs);

    gfx.setCursor(t.tx, t.ty + 9);
    gfx.print("FL");
    gfx.print(t.fl);
    gfx.print(" ");
    gfx.print(t.kts);

    if (!t.route.isEmpty()) {
      gfx.setCursor(t.tx, t.ty + 18);
      gfx.print(t.route);
    }

    addCurrentRegion(t.tx - 1, t.ty - 1, t.tagW + 2, t.tagH + 2);
  }
}

void drawHud(Adafruit_GFX& gfx) {
  gfx.setTextSize(1);
  gfx.setTextColor(radarGreen(180));
  gfx.setCursor(8, 6);
  gfx.print("R:");
  gfx.print(cfg.radiusKm, 0);
  gfx.print("km");

  gfx.setCursor(8, 18);
  gfx.print("AC:");
  gfx.print(aircraftCount);

  addCurrentRegion(6, 4, 80, 24);
}

void drawRadarFrame() {
  if (dynamicDirty) {
    if (canvasReady) {
      canvas.fillScreen(0x0000);
      drawGrid(canvas);
      drawAirports(canvas);

      currentDynamicRegionCount = 0;
      drawAircraft(canvas);
      drawHud(canvas);

      if (firstFrame) {
        tft.drawRGBBitmap(0, 0, canvas.getBuffer(), SCREEN_W, SCREEN_H);
        firstFrame = false;
      } else {
        for (int i = 0; i < prevDynamicRegionCount; i++) {
          RectRegion r = prevDynamicRegions[i];
          pushCanvasRegion(r.x, r.y, r.w, r.h);
        }
        for (int i = 0; i < currentDynamicRegionCount; i++) {
          RectRegion r = currentDynamicRegions[i];
          pushCanvasRegion(r.x, r.y, r.w, r.h);
        }
      }

      prevDynamicRegionCount = currentDynamicRegionCount;
      for (int i = 0; i < currentDynamicRegionCount; i++) {
        prevDynamicRegions[i] = currentDynamicRegions[i];
      }
    } else {
      // Memory allocation failed fallback
      tft.fillScreen(0x0000);
      drawGrid(tft);
      drawAirports(tft);
      drawAircraft(tft);
      drawHud(tft);
    }
    dynamicDirty = false;
  }
}

void setupDisplay() {
  if (PIN_TFT_BL >= 0) {
    pinMode(PIN_TFT_BL, OUTPUT);
    digitalWrite(PIN_TFT_BL, HIGH);
  }

  // Pass -1 to hardware SPI to allow Adafruit_GFX to manually manage the CS pin
  SPI.begin(PIN_TFT_SCK, -1, PIN_TFT_MOSI, -1);
  tft.begin();
  tft.setRotation(0);
  tft.fillScreen(0x0000);
  tft.setTextWrap(false);

  canvasReady = (canvas.getBuffer() != nullptr);
  if (canvasReady) {
    canvas.setTextWrap(false);
  } else {
    Serial.println("[WARN] Framebuffer alloc failed, using direct mode");
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);
  debugLog("Boot start");

  initAdsbCache();
  setupColors();
  loadConfig();
  loadAirportsFromFlash();
  printConfigSummary();
  setupDisplay();
  setupBLE();

  connectWiFi();

  checkForUpdates();
  // Fetch from ADSB.fi on boot
  if (fetchAdsbFi()) {
    dynamicDirty = true;
  }
  if (updateAirportsIfMoved()) {
    dynamicDirty = true;
  }
  drawRadarFrame();
}

void cleanAdsbCache() {
  uint32_t now = millis();
  for (int i = 0; i < MAX_ADSB_CACHE; i++) {
    if (adsbCache[i].used && (now - adsbCache[i].lastSeenMs > 60000UL)) {
      adsbCache[i].used = false; // Plane left area or API stopped tracking it
    }
  }
}

void loop() {
  uint32_t now = millis();

  if (now - lastFetchMs > FETCH_INTERVAL_MS) {
    lastFetchMs = now;
    if (fetchAdsbFi()) {
      dynamicDirty = true;
    }
    if (updateAirportsIfMoved()) {
      dynamicDirty = true;
    }
  }

  if (now - lastBlipDrawMs > BLIP_REFRESH_INTERVAL_MS) {
    lastBlipDrawMs = now;
    dynamicDirty = true;
  }

  if (now - lastAdsbLookupMs > ADSB_LOOKUP_SPACING_MS && now - lastFetchMs < FETCH_INTERVAL_MS) {
    lastAdsbLookupMs = now;
    lookupOnePendingAdsbdbRoute();
  }

  cleanAdsbCache();

  drawRadarFrame();

  delay(2);
}