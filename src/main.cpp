#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
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
constexpr uint32_t FETCH_INTERVAL_MS = 10000;
constexpr uint32_t BLIP_REFRESH_INTERVAL_MS = 10000;
constexpr int MAX_AIRCRAFT = 80;
constexpr int MAX_TAGS_ON_SCREEN = 18;
constexpr int MAX_DYNAMIC_REGIONS = 160;
constexpr int MAX_ADSB_CACHE = 96;
constexpr uint32_t ADSB_LOOKUP_SPACING_MS = 1200;

Adafruit_GC9A01A tft(PIN_TFT_CS, PIN_TFT_DC, PIN_TFT_RST);
GFXcanvas16 baseCanvas(SCREEN_W, SCREEN_H);
Preferences prefs;

struct Config {
  String ssid;
  String pass;
  float centerLat;
  float centerLon;
  float radiusKm;
  float speedCutoffKts;
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

Aircraft aircraft[MAX_AIRCRAFT];
int aircraftCount = 0;

uint32_t lastFetchMs = 0;
uint32_t lastBlipDrawMs = 0;
bool wifiConnected = false;
bool baseCanvasReady = false;
bool staticBaseDirty = true;
bool dynamicDirty = true;
bool bleClientConnected = false;
int lastHttpCode = 0;
uint32_t successfulFetchCount = 0;
uint32_t fetchEpoch = 0;
String lastStatusText = "Boot";
uint32_t lastNoWiFiStatusMs = 0;
uint32_t lastWiFiRetryMs = 0;
uint32_t lastAdsbLookupMs = 0;

struct AdsbCacheEntry {
  bool used;
  String modeS;
  String routeIata;
  bool lookedUp;
  bool hasRoute;
  uint32_t lastLookupMs;
};

AdsbCacheEntry adsbCache[MAX_ADSB_CACHE];

struct RectRegion {
  int16_t x;
  int16_t y;
  int16_t w;
  int16_t h;
};

RectRegion prevDynamicRegions[MAX_DYNAMIC_REGIONS];
int prevDynamicRegionCount = 0;
RectRegion currentDynamicRegions[MAX_DYNAMIC_REGIONS];
int currentDynamicRegionCount = 0;

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
String authToken;
uint32_t authTokenExpiresAtMs = 0;
bool verboseLogging = false;

void publishStatus(const String& text);
String urlEncode(const String& in);

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
    "A388", "A359", "A35K", "A346", "A345", "A333", "A332", "A339", "A343",
    "B744", "B748", "B77W", "B772", "B773", "B788", "B789", "B78X", "B763",
    "MD11", "DC10", "L101", "C17", "C5M", "C5", "A400", "E3TF", "E8", "K35R",
    "KC46", "KC10", "R135", "P8", "E767", "F15", "F16", "F18", "F22", "F35",
    "RFAL", "EUFI", "JAS3", "SU27", "SU30", "SU34", "M346"
  };

  for (size_t i = 0; i < sizeof(exactTypes) / sizeof(exactTypes[0]); i++) {
    if (t == exactTypes[i]) return true;
  }

  const char* prefixTypes[] = {
    "A3", "A2", "A1", "B7", "B8", "B9", "E17", "E19", "E2", "C17", "C5", "KC",
    "IL7", "AN12", "AN22", "F1", "F2", "F3", "SU", "MIG"
  };

  for (size_t i = 0; i < sizeof(prefixTypes) / sizeof(prefixTypes[0]); i++) {
    if (t.startsWith(prefixTypes[i])) return true;
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
  uint32_t oldestTs = adsbCache[0].lastLookupMs;
  for (int i = 1; i < MAX_ADSB_CACHE; i++) {
    if (adsbCache[i].lastLookupMs < oldestTs) {
      oldestTs = adsbCache[i].lastLookupMs;
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

    if (idx < 0) idx = reserveAdsbCacheIndex();
    adsbCache[idx].used = true;
    adsbCache[idx].modeS = modeS;

    String routeIata = "";
    bool requestOk = fetchAdsbdbRoute(modeS, aircraft[i].callsign, routeIata);

    adsbCache[idx].lookedUp = true;
    adsbCache[idx].lastLookupMs = millis();
    adsbCache[idx].routeIata = routeIata;
    adsbCache[idx].hasRoute = requestOk && !routeIata.isEmpty();

    if (adsbCache[idx].hasRoute) {
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

String base64Encode(const String& in) {
  static const char* table = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  String out;
  size_t len = in.length();
  out.reserve(((len + 2) / 3) * 4);

  for (size_t i = 0; i < len; i += 3) {
    uint32_t octetA = (uint8_t)in[i];
    uint32_t octetB = (i + 1 < len) ? (uint8_t)in[i + 1] : 0;
    uint32_t octetC = (i + 2 < len) ? (uint8_t)in[i + 2] : 0;

    uint32_t triple = (octetA << 16) | (octetB << 8) | octetC;
    out += table[(triple >> 18) & 0x3F];
    out += table[(triple >> 12) & 0x3F];
    out += (i + 1 < len) ? table[(triple >> 6) & 0x3F] : '=';
    out += (i + 2 < len) ? table[triple & 0x3F] : '=';
  }

  return out;
}

bool refreshAuthToken() {
  if (cfg.apiClientId.isEmpty() || cfg.apiClientSecret.isEmpty()) {
    authToken = "";
    authTokenExpiresAtMs = 0;
    return false;
  }

  String clientId = cfg.apiClientId;
  String clientSecret = cfg.apiClientSecret;
  clientId.trim();
  clientSecret.trim();
  if (clientId.isEmpty() || clientSecret.isEmpty()) {
    authToken = "";
    authTokenExpiresAtMs = 0;
    publishStatus("Token creds empty");
    return false;
  }

  authToken = "";
  authTokenExpiresAtMs = 0;

  auto tryTokenRequest = [&](bool useBasicAuth) -> bool {
    WiFiClientSecure client;
    client.setInsecure();

    HTTPClient http;
    http.setConnectTimeout(6000);
    http.setTimeout(9000);

    if (!http.begin(client, "https://auth.opensky-network.org/auth/realms/opensky-network/protocol/openid-connect/token")) {
      publishStatus("Token begin failed");
      return false;
    }

    http.addHeader("Content-Type", "application/x-www-form-urlencoded");
    http.addHeader("Accept", "application/json");

    String body;
    if (useBasicAuth) {
      String basic = base64Encode(clientId + ":" + clientSecret);
      http.addHeader("Authorization", "Basic " + basic);
      body = "grant_type=client_credentials";
      debugLog("Token request method=basic_auth");
    } else {
      body = "grant_type=client_credentials&client_id=" + urlEncode(clientId) +
             "&client_secret=" + urlEncode(clientSecret);
      debugLog("Token request method=form_fields");
    }

    int code = http.POST(body);
    String payload = http.getString();
    http.end();

    if (verboseLogging) {
      Serial.println("[VRB] Token response payload:");
      Serial.println(payload);
    }

    if (code != HTTP_CODE_OK) {
      String preview = payload.substring(0, 160);
      preview.replace("\n", " ");
      preview.replace("\r", " ");
      debugLog("Token HTTP " + String(code) + " payload='" + preview + "'");

      JsonDocument errDoc;
      DeserializationError derr = deserializeJson(errDoc, payload);
      if (!derr && !errDoc["error"].isNull()) {
        String errCode = String((const char*)errDoc["error"]);
        String errDesc = errDoc["error_description"].isNull() ? "" : String((const char*)errDoc["error_description"]);
        errDesc.trim();
        if (errDesc.length() > 42) errDesc = errDesc.substring(0, 42);
        publishStatus("Token " + String(code) + " " + errCode + (errDesc.isEmpty() ? "" : (":" + errDesc)));
      } else {
        publishStatus("Token HTTP " + String(code));
      }
      return false;
    }

    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, payload);
    if (err || doc["access_token"].isNull()) {
      publishStatus("Token parse err");
      return false;
    }

    authToken = String((const char*)doc["access_token"]);
    uint32_t expiresIn = doc["expires_in"].isNull() ? 1800U : doc["expires_in"].as<uint32_t>();
    uint32_t margin = (expiresIn > 60U) ? 60U : 5U;
    authTokenExpiresAtMs = millis() + (expiresIn - margin) * 1000U;
    debugLog("Token OK exp_in=" + String(expiresIn));
    return true;
  };

  if (tryTokenRequest(false)) return true;
  return tryTokenRequest(true);
}

bool ensureAuthToken() {
  if (cfg.apiClientId.isEmpty() || cfg.apiClientSecret.isEmpty()) return false;
  if (!authToken.isEmpty() && millis() < authTokenExpiresAtMs) return true;
  return refreshAuthToken();
}

void saveConfig() {
  prefs.begin("radar", false);
  prefs.putString("ssid", cfg.ssid);
  prefs.putString("pass", cfg.pass);
  prefs.putFloat("clat", cfg.centerLat);
  prefs.putFloat("clon", cfg.centerLon);
  prefs.putFloat("rad", cfg.radiusKm);
  prefs.putFloat("spd", cfg.speedCutoffKts);
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
  cfg.apiClientId = prefs.getString("cid", "");
  cfg.apiClientSecret = prefs.getString("csec", "");
  prefs.end();

  cfg.radiusKm = clampf(cfg.radiusKm, 5.0f, 150.0f);
  cfg.speedCutoffKts = clampf(cfg.speedCutoffKts, 0.0f, 700.0f);
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
      authToken = "";
      authTokenExpiresAtMs = 0;
      saveConfig();
      publishStatus("client_id saved");
    }
    else if (key_ == "client_secret") {
      cfg.apiClientSecret = v;
      authToken = "";
      authTokenExpiresAtMs = 0;
      saveConfig();
      publishStatus("client_secret saved");
    }
    else if (key_ == "cmd") {
      String cmd = v;
      cmd.toLowerCase();

      if (cmd == "save") {
        saveConfig();
        authToken = "";
        authTokenExpiresAtMs = 0;
        printConfigSummary();
        publishStatus("Config saved");
      } else if (cmd == "apply") {
        saveConfig();
        authToken = "";
        authTokenExpiresAtMs = 0;
        printConfigSummary();
        connectWiFi();
      } else if (cmd == "auth") {
        if (cfg.apiClientId.isEmpty() || cfg.apiClientSecret.isEmpty()) {
          publishStatus("Set client_id/secret first");
        } else if (refreshAuthToken()) {
          publishStatus("Auth token active");
        } else {
          publishStatus("Auth token failed");
        }
      } else if (cmd == "verbose" || cmd == "verbose on") {
        verboseLogging = true;
        publishStatus("Verbose ON");
      } else if (cmd == "verbose off") {
        verboseLogging = false;
        publishStatus("Verbose OFF");
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

bool fetchOpenSky() {
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
        debugLog("OpenSky skipped, WiFi not connected (" + wifiStatusToText(WiFi.status()) + ")");
        publishStatus("No WiFi for OpenSky");
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

  float latDelta = cfg.radiusKm / 111.0f;
  float lonScale = cosf(degToRad(cfg.centerLat));
  if (fabs(lonScale) < 0.2f) lonScale = 0.2f;
  float lonDelta = cfg.radiusKm / (111.0f * lonScale);

  float lamin = cfg.centerLat - latDelta;
  float lamax = cfg.centerLat + latDelta;
  float lomin = cfg.centerLon - lonDelta;
  float lomax = cfg.centerLon + lonDelta;

  String url = "https://opensky-network.org/api/states/all?lamin=" + String(lamin, 5) +
               "&lomin=" + String(lomin, 5) +
               "&lamax=" + String(lamax, 5) +
               "&lomax=" + String(lomax, 5) +
               "&extended=1";
  debugLog("OpenSky GET " + url);

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

  bool authConfigured = !cfg.apiClientId.isEmpty() && !cfg.apiClientSecret.isEmpty();
  bool authEnabled = ensureAuthToken();

  if (authConfigured && !authEnabled) {
    debugLog("OpenSky auth unavailable, falling back to anonymous");
    publishStatus("Auth unavailable, anonymous mode");
  }

  if (authEnabled && !authToken.isEmpty()) {
    http.addHeader("Authorization", "Bearer " + authToken);
    debugLog("OpenSky auth: Bearer token attached");
  } else {
    debugLog("OpenSky auth: anonymous request");
  }

  int code = http.GET();
  if (code == HTTP_CODE_UNAUTHORIZED && authEnabled && refreshAuthToken()) {
    debugLog("OpenSky auth: 401 received, refreshing token and retrying");
    http.end();

    WiFiClientSecure retryClient;
    retryClient.setInsecure();
    HTTPClient retry;
    retry.setConnectTimeout(7000);
    retry.setTimeout(9000);
    retry.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
    if (!retry.begin(retryClient, url)) {
      publishStatus("HTTP begin failed");
      return false;
    }
    if (!authToken.isEmpty()) {
      retry.addHeader("Authorization", "Bearer " + authToken);
      debugLog("OpenSky auth: refreshed Bearer token attached");
    }
    code = retry.GET();
    lastHttpCode = code;
    if (code != HTTP_CODE_OK) {
      publishStatus("OpenSky HTTP " + String(code));
      retry.end();
      return false;
    }
    String payload = retry.getString();
    debugLog("OpenSky payload bytes=" + String(payload.length()));
    if (verboseLogging) {
      Serial.println("[VRB] OpenSky response payload (retry):");
      Serial.println(payload);
    }

    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, payload);
    retry.end();

    if (err) {
      String preview = payload.substring(0, 120);
      preview.replace("\n", " ");
      preview.replace("\r", " ");
      debugLog("JSON err=" + String(err.c_str()) + " preview='" + preview + "'");
      publishStatus("JSON err: " + String(err.c_str()));
      return false;
    }

    JsonArray states = doc["states"].as<JsonArray>();
    if (states.isNull()) {
      aircraftCount = 0;
      return true;
    }

    int count = 0;
    for (JsonVariant v : states) {
      if (count >= MAX_AIRCRAFT) break;

      JsonArray s = v.as<JsonArray>();
      if (s.isNull() || s.size() < 11) continue;
      if (s[5].isNull() || s[6].isNull()) continue;

      float lon = s[5].as<float>();
      float lat = s[6].as<float>();
      float dist = greatCircleKm(cfg.centerLat, cfg.centerLon, lat, lon);
      if (dist > cfg.radiusKm) continue;

      String callsign = s[1].isNull() ? "" : String((const char*)s[1]);
      callsign.trim();
      if (callsign.isEmpty()) callsign = "UNK";

      String modeS = s[0].isNull() ? "" : String((const char*)s[0]);
      modeS.toUpperCase();
      modeS.trim();
      if (modeS.isEmpty()) continue;

      float trackDeg = s[10].isNull() ? 0.0f : s[10].as<float>();
      float velocity = s[9].isNull() ? 0.0f : s[9].as<float>();
      if (mpsToKnots(velocity) <= cfg.speedCutoffKts) continue;

      float altitudeM = 0.0f;
      if (!s[13].isNull()) altitudeM = s[13].as<float>();
      else if (!s[7].isNull()) altitudeM = s[7].as<float>();

      aircraft[count].callsign = callsign;
      aircraft[count].modeS = modeS;
      aircraft[count].icaoType = "";
      aircraft[count].routeIata = "";
      String cachedRoute;
      if (getCachedRoute(modeS, cachedRoute)) aircraft[count].routeIata = cachedRoute;
      aircraft[count].lat = lat;
      aircraft[count].lon = lon;
      aircraft[count].trackDeg = trackDeg;
      aircraft[count].velocity = velocity;
      aircraft[count].altitudeM = altitudeM;
      count++;
    }

    aircraftCount = count;
    applyCachedRoutesToCurrentAircraft();
    successfulFetchCount++;
    debugLog("OpenSky parsed aircraft=" + String(aircraftCount));
    publishStatus("Aircraft: " + String(aircraftCount));
    dynamicDirty = true;
    return true;
  }

  lastHttpCode = code;
  if (code != HTTP_CODE_OK) {
    publishStatus("OpenSky HTTP " + String(code));
    http.end();
    return false;
  }

  String payload = http.getString();
  debugLog("OpenSky payload bytes=" + String(payload.length()));
  if (verboseLogging) {
    Serial.println("[VRB] OpenSky response payload:");
    Serial.println(payload);
  }

  JsonDocument doc;
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

  JsonArray states = doc["states"].as<JsonArray>();
  if (states.isNull()) {
    aircraftCount = 0;
    return true;
  }

  int count = 0;
  for (JsonVariant v : states) {
    if (count >= MAX_AIRCRAFT) break;

    JsonArray s = v.as<JsonArray>();
    if (s.isNull() || s.size() < 11) continue;

    if (s[5].isNull() || s[6].isNull()) continue;

    float lon = s[5].as<float>();
    float lat = s[6].as<float>();
    float dist = greatCircleKm(cfg.centerLat, cfg.centerLon, lat, lon);
    if (dist > cfg.radiusKm) continue;

    String callsign = s[1].isNull() ? "" : String((const char*)s[1]);
    callsign.trim();
    if (callsign.isEmpty()) callsign = "UNK";

    String modeS = s[0].isNull() ? "" : String((const char*)s[0]);
    modeS.toUpperCase();
    modeS.trim();
    if (modeS.isEmpty()) continue;

    float trackDeg = s[10].isNull() ? 0.0f : s[10].as<float>();
    float velocity = s[9].isNull() ? 0.0f : s[9].as<float>();
    if (mpsToKnots(velocity) <= cfg.speedCutoffKts) continue;

    float altitudeM = 0.0f;
    if (!s[13].isNull()) altitudeM = s[13].as<float>();
    else if (!s[7].isNull()) altitudeM = s[7].as<float>();

    aircraft[count].callsign = callsign;
    aircraft[count].modeS = modeS;
    aircraft[count].icaoType = "";
    aircraft[count].routeIata = "";
    String cachedRoute;
    if (getCachedRoute(modeS, cachedRoute)) aircraft[count].routeIata = cachedRoute;
    aircraft[count].lat = lat;
    aircraft[count].lon = lon;
    aircraft[count].trackDeg = trackDeg;
    aircraft[count].velocity = velocity;
    aircraft[count].altitudeM = altitudeM;

    count++;
  }

  aircraftCount = count;
  applyCachedRoutesToCurrentAircraft();
  successfulFetchCount++;
  debugLog("OpenSky parsed aircraft=" + String(aircraftCount));
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

bool rectOverlapsAircraftMarker(const RectRegion& r, int markerX, int markerY) {
  RectRegion markerBox = {
    (int16_t)(markerX - 5),
    (int16_t)(markerY - 5),
    11,
    11
  };
  return rectsOverlap(r, markerBox);
}

void addCurrentRegion(int x, int y, int w, int h) {
  if (currentDynamicRegionCount >= MAX_DYNAMIC_REGIONS) return;
  if (w <= 0 || h <= 0) return;

  int x0 = max(0, x);
  int y0 = max(0, y);
  int x1 = min(SCREEN_W, x + w);
  int y1 = min(SCREEN_H, y + h);
  if (x1 <= x0 || y1 <= y0) return;

  currentDynamicRegions[currentDynamicRegionCount++] = {
    (int16_t)x0,
    (int16_t)y0,
    (int16_t)(x1 - x0),
    (int16_t)(y1 - y0)
  };
}

void restorePreviousDynamicRegions() {
  if (!baseCanvasReady) return;

  const uint16_t* baseBuf = baseCanvas.getBuffer();
  static uint16_t lineBuf[SCREEN_W];

  for (int i = 0; i < prevDynamicRegionCount; i++) {
    RectRegion r = prevDynamicRegions[i];
    for (int yy = r.y; yy < r.y + r.h; yy++) {
      int src = yy * SCREEN_W + r.x;
      for (int xx = 0; xx < r.w; xx++) {
        lineBuf[xx] = baseBuf[src + xx];
      }
      tft.drawRGBBitmap(r.x, yy, lineBuf, r.w, 1);
    }
  }
}

bool pickTagPosition(int blipX,
                     int blipY,
                     int tagW,
                     int tagH,
                     const int markerXs[],
                     const int markerYs[],
                     const bool markerValid[],
                     int markerCount,
                     const RectRegion placedTags[],
                     int placedCount,
                     int& outX,
                     int& outY,
                     bool& outNeedsLeader) {
  auto isValid = [&](int x, int y) {
    RectRegion cand = {(int16_t)x, (int16_t)y, (int16_t)tagW, (int16_t)tagH};

    if (cand.x < 0 || cand.y < 0 || cand.x + cand.w > SCREEN_W || cand.y + cand.h > SCREEN_H) {
      return false;
    }

    for (int j = 0; j < placedCount; j++) {
      if (rectsOverlap(cand, placedTags[j])) return false;
    }

    for (int j = 0; j < markerCount; j++) {
      if (!markerValid[j]) continue;
      if (rectOverlapsAircraftMarker(cand, markerXs[j], markerYs[j])) return false;
    }

    return true;
  };

  // Prefer right/left of marker first.
  const int preferredCount = 4;
  int px[preferredCount] = {blipX + 8, blipX - tagW - 8, blipX + 8, blipX - tagW - 8};
  int py[preferredCount] = {blipY - tagH / 2, blipY - tagH / 2, blipY - tagH - 4, blipY + 4};

  for (int i = 0; i < preferredCount; i++) {
    if (!isValid(px[i], py[i])) continue;
    outX = px[i];
    outY = py[i];
    outNeedsLeader = false;
    return true;
  }

  // Fallback: find nearest valid position anywhere on screen.
  int bestX = -1;
  int bestY = -1;
  uint32_t bestDist2 = 0xFFFFFFFFu;
  const int step = 4;

  for (int y = 0; y <= SCREEN_H - tagH; y += step) {
    for (int x = 0; x <= SCREEN_W - tagW; x += step) {
      if (!isValid(x, y)) continue;
      int dx = (x + tagW / 2) - blipX;
      int dy = (y + tagH / 2) - blipY;
      uint32_t d2 = (uint32_t)(dx * dx + dy * dy);
      if (d2 < bestDist2) {
        bestDist2 = d2;
        bestX = x;
        bestY = y;
      }
    }
  }

  if (bestX >= 0) {
    outX = bestX;
    outY = bestY;
    outNeedsLeader = true;
    return true;
  }

  return false;
}

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

    gfx.fillCircle(x, y, 2, radarGreen(255));
    gfx.drawCircle(x, y, 4, radarGreen(120));
    addCurrentRegion(x - 5, y - 5, 11, 11);

    float hdg = degToRad(aircraft[i].trackDeg);
    int hx = x + (int)(sinf(hdg) * 6.0f);
    int hy = y - (int)(cosf(hdg) * 6.0f);
    gfx.drawLine(x, y, hx, hy, radarGreen(180));
    int lx = min(x, hx) - 1;
    int ly = min(y, hy) - 1;
    addCurrentRegion(lx, ly, abs(hx - x) + 3, abs(hy - y) + 3);
  }

  // Pass 2: place labels with overlap avoidance.
  RectRegion placedTags[MAX_TAGS_ON_SCREEN];
  int placedTagCount = 0;
  int tagsDrawn = 0;

  for (int i = 0; i < aircraftCount; i++) {
    if (!markerValid[i]) continue;
    int x = markerX[i];
    int y = markerY[i];

    if (tagsDrawn < MAX_TAGS_ON_SCREEN) {
      String cs = aircraft[i].callsign;
      if (cs.isEmpty()) cs = "UNK";
      if (cs.length() > 8) cs = cs.substring(0, 8);

      int fl = metersToFlightLevel(aircraft[i].altitudeM);
      int kts = (int)(mpsToKnots(aircraft[i].velocity) + 0.5f);
      String route = aircraft[i].routeIata;
      route.trim();
      bool hasRoute = !route.isEmpty();

      int tagW = 104;
      int tagH = hasRoute ? 27 : 18;
      int tx = 0;
      int ty = 0;
      bool needsLeader = false;

      if (!pickTagPosition(x,
                           y,
                           tagW,
                           tagH,
                           markerX,
                           markerY,
                           markerValid,
                           aircraftCount,
                           placedTags,
                           placedTagCount,
                           tx,
                           ty,
                           needsLeader)) {
        continue;
      }

      gfx.setTextColor(radarGreen(210));

      gfx.setCursor(tx, ty);
      gfx.print(cs);

      gfx.setCursor(tx, ty + 9);
      gfx.print("FL");
      gfx.print(fl);
      gfx.print(" ");
      gfx.print(kts);

      if (hasRoute) {
        gfx.setCursor(tx, ty + 18);
        gfx.print(route);
      }

      if (needsLeader) {
        int ex = x;
        if (ex < tx) ex = tx;
        if (ex > tx + tagW - 1) ex = tx + tagW - 1;
        int ey = y;
        if (ey < ty) ey = ty;
        if (ey > ty + tagH - 1) ey = ty + tagH - 1;
        gfx.drawLine(x, y, ex, ey, radarGreen(90));
        int lx = min(x, ex) - 1;
        int ly = min(y, ey) - 1;
        addCurrentRegion(lx, ly, abs(ex - x) + 3, abs(ey - y) + 3);
      }

      addCurrentRegion(tx - 1, ty - 1, tagW + 2, tagH + 2);
      if (placedTagCount < MAX_TAGS_ON_SCREEN) {
        placedTags[placedTagCount++] = {(int16_t)tx, (int16_t)ty, (int16_t)tagW, (int16_t)tagH};
      }

      tagsDrawn++;
    }
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

  addCurrentRegion(6, 4, 64, 20);
}

void drawStaticBase(Adafruit_GFX& gfx) {
  gfx.fillScreen(0x0000);
  drawGrid(gfx);
}

void drawDynamicLayer() {
  if (baseCanvasReady) {
    restorePreviousDynamicRegions();
  } else {
    drawStaticBase(tft);
  }

  currentDynamicRegionCount = 0;
  drawAircraft(tft);
  drawHud(tft);

  for (int i = 0; i < currentDynamicRegionCount; i++) {
    prevDynamicRegions[i] = currentDynamicRegions[i];
  }
  prevDynamicRegionCount = currentDynamicRegionCount;
}

void drawRadarFrame() {
  if (staticBaseDirty) {
    if (baseCanvasReady) {
      drawStaticBase(baseCanvas);
      tft.drawRGBBitmap(0, 0, baseCanvas.getBuffer(), SCREEN_W, SCREEN_H);
    } else {
      drawStaticBase(tft);
    }
    prevDynamicRegionCount = 0;
    staticBaseDirty = false;
    dynamicDirty = true;
  }

  if (dynamicDirty) {
    drawDynamicLayer();
    dynamicDirty = false;
  }
}

void setupDisplay() {
  if (PIN_TFT_BL >= 0) {
    pinMode(PIN_TFT_BL, OUTPUT);
    digitalWrite(PIN_TFT_BL, HIGH);
  }

  SPI.begin(PIN_TFT_SCK, -1, PIN_TFT_MOSI, PIN_TFT_CS);
  tft.begin();
  tft.setRotation(0);
  tft.fillScreen(0x0000);

  baseCanvasReady = (baseCanvas.getBuffer() != nullptr);
  if (!baseCanvasReady) {
    Serial.println("[WARN] Framebuffer alloc failed, using direct mode");
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);
  debugLog("Boot start");

  loadConfig();
  printConfigSummary();
  setupDisplay();
  setupBLE();

  connectWiFi();
  if (fetchOpenSky()) {
    dynamicDirty = true;
  }
  drawRadarFrame();
}

void loop() {
  uint32_t now = millis();

  if (now - lastFetchMs > FETCH_INTERVAL_MS) {
    lastFetchMs = now;
    if (fetchOpenSky()) {
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

  drawRadarFrame();

  delay(2);
}
