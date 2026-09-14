// ================================================================
//  poll_engine.cpp  –  services one due point per call so loop()
//  stays responsive to the Modbus TCP server and the web frontend
//  between RS485 round-trips.
// ================================================================
#include "poll_engine.h"

// Minimum gap enforced between two consecutive RTU transactions, regardless
// of how many points are due at once — keeps the bus from being hammered
// back-to-back and gives slow RTU slaves time to turn around.
static const unsigned long MIN_SCAN_GAP_MS = 20;

static unsigned long lastScanMs = 0;

void pollEngineInit() {
  unsigned long now = millis();
  for (uint8_t i = 0; i < MAX_POINTS; i++) {
    // Stagger initial due-times so an all-enabled config doesn't burst
    // 16 RTU requests back-to-back the moment the bus comes up.
    pointState[i].nextDueMs = now + (unsigned long)i * 50UL;
  }
  lastScanMs = now;
}

void pollEngineLoop() {
  unsigned long now = millis();
  if (now - lastScanMs < MIN_SCAN_GAP_MS) return;

  int8_t dueIdx = -1;
  uint32_t earliestDue = 0;

  for (uint8_t i = 0; i < MAX_POINTS; i++) {
    if (!points[i].enable) continue;
    if ((int32_t)(now - pointState[i].nextDueMs) < 0) continue;   // not due yet
    if (dueIdx < 0 || (int32_t)(pointState[i].nextDueMs - earliestDue) < 0) {
      dueIdx = (int8_t)i;
      earliestDue = pointState[i].nextDueMs;
    }
  }

  if (dueIdx < 0) return;   // nothing due — let loop() service TCP/web instead

  PointConfig  &cfg = points[dueIdx];
  PointRuntime &rt  = pointState[dueIdx];

  // Elapsed time is the key diagnostic here: if a "failed" transaction took
  // roughly the full response-timeout duration, it genuinely waited and got
  // nothing back (a real comms/timing problem). If it took ~0ms, the timeout
  // check itself isn't working and we're not really waiting at all — a
  // different bug entirely. Logging successes too (temporarily, while this
  // is being diagnosed) is the only way to see the true success/fail ratio;
  // the Dashboard doesn't auto-refresh so it can't be trusted for this, and
  // the Modbus TCP server always answers *something* well-formed from
  // whatever's cached regardless of whether RTU polling is working at all.
  unsigned long tStart = millis();
  bool ok = mbRtuReadPoint(cfg, rt);
  unsigned long elapsed = millis() - tStart;

  if (ok) {
    addLog(String("[POLL] ") + cfg.name + " OK val=" + floatToStr(rt.value, 3) +
           " (" + elapsed + "ms)");
  } else {
    addLog(String("[POLL] ") + cfg.name + " read failed: " +
           mbRtuFailReasonText(mbRtuLastFailReason()) +
           " (got " + mbRtuLastFailLen() + "B, fail#" + rt.failCount +
           ", took " + elapsed + "ms)");
  }

  rt.nextDueMs = now + (uint32_t)cfg.pollIntervalMs;
  lastScanMs = now;
}
