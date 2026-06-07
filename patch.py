import sys

filepath = r"c:\Users\fuwin\Documents\Radar\src\main.cpp"
with open(filepath, 'r', encoding='utf-8') as f:
    content = f.read()

old_cache_entry = '''struct AdsbCacheEntry {
  bool used;
  String modeS;
  String routeIata;
  bool lookedUp;
  bool hasRoute;
  uint32_t lastLookupMs;
};'''

new_cache_entry = '''struct AdsbCacheEntry {
  bool used;
  String modeS;
  String routeIata;
  bool lookedUp;
  bool hasRoute;
  uint32_t lastLookupMs;
  bool hasLabelAngle;
  float targetLabelAngle;
  float currentLabelAngle;
};'''

if old_cache_entry in content:
    content = content.replace(old_cache_entry, new_cache_entry)
else:
    print("Could not find AdsbCacheEntry")

old_pick_tag = '''bool pickTagPosition(int blipX,
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
}'''

new_functions = '''struct TagToDraw {
  int tx;
  int ty;
  int tagW;
  int tagH;
  String cs;
  int fl;
  int kts;
  String route;
};

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
  }
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

void drawThickLine(Adafruit_GFX& gfx, int x0, int y0, int x1, int y1, uint16_t color) {
  gfx.drawLine(x0, y0, x1, y1, color);
  gfx.drawLine(x0 + 1, y0, x1 + 1, y1, color);
  gfx.drawLine(x0, y0 + 1, x1, y1 + 1, color);
}'''

if old_pick_tag in content:
    content = content.replace(old_pick_tag, new_functions)
else:
    print("Could not find pickTagPosition")

with open(filepath, 'w', encoding='utf-8') as f:
    f.write(content)

print("File updated successfully.")
