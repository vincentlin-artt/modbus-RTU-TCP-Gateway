// ================================================================
//  modbus_tcp_server.cpp  –  Modbus TCP server, data-concentrator side
//
//  Reads are served straight from the pollEngine's cache (pointState[]) —
//  no RS485 traffic on the read path, so many TCP clients can poll at once
//  without waiting on each other or on the RTU bus.
//
//  EthernetServer::available() (nvtEther, lwIP-backed) already tracks up
//  to MAX_CLIENT=32 accepted sockets internally and hands back whichever
//  one currently has data — so servicing >=4 concurrent TCP clients needs
//  no extra socket bookkeeping here, just calling available() in a loop.
//
//  Writes (FC05/06/16) are forwarded live to the RS485 bus via
//  modbus_rtu's mbRtuWriteRaw() and share the same loop() thread as the
//  poll engine, so they're automatically serialized with it — there is
//  only ever one RTU transaction in flight at a time, which is a hard
//  requirement of the RS485 bus anyway.
//
//  Wire convention: TCP-side registers/coils are raw pass-through of the
//  RTU device's registers — what a Modbus TCP client reads/writes here is
//  byte-identical to what goes out/came in over RS485. This matches how
//  commercial Modbus RTU<->TCP gateways behave and avoids ambiguity about
//  which end applies scale/byte-order. format/byteOrder/scale are only
//  used locally (web dashboard, RTU-side interpretation).
// ================================================================
#include "modbus_tcp_server.h"

static EthernetServer *mbServer = nullptr;

// ---- dial-out ("Client") mode: single persistent outbound connection ----
// Modbus-protocol roles don't change — this device still answers requests
// via the exact same buildResponse() path as listen mode — only who opened
// the TCP socket differs. See config.h's mbTcpClientMode.
static EthernetClient g_outClient;
static unsigned long  g_lastConnectAttemptMs = 0;
#define MB_RECONNECT_INTERVAL_MS 10000UL

// ---- best-effort "recently active" client tracking for the dashboard ----
// EthernetServer doesn't expose a client count, so we fingerprint by
// remote IP+port and age entries out after MB_TRACK_WINDOW_MS of silence.
// This is an approximation, not an exact live socket count.
#define MB_TRACK_MAX        8
#define MB_TRACK_WINDOW_MS  15000UL
struct ClientTrack { bool used; uint32_t ip; uint16_t port; uint32_t lastSeenMs; };
static ClientTrack g_track[MB_TRACK_MAX];

static void trackClient(EthernetClient &c) {
  uint32_t ip = (uint32_t)c.remoteIP();
  uint16_t port = c.remotePort();
  uint32_t now = millis();
  int freeSlot = -1;
  for (int i = 0; i < MB_TRACK_MAX; i++) {
    if (g_track[i].used && g_track[i].ip == ip && g_track[i].port == port) {
      g_track[i].lastSeenMs = now;
      return;
    }
    if (!g_track[i].used && freeSlot < 0) freeSlot = i;
  }
  if (freeSlot < 0) freeSlot = 0;   // table full: recycle slot 0 (best-effort)
  g_track[freeSlot] = { true, ip, port, now };
}

uint16_t mbTcpActiveClientCount() {
  if (mbTcpClientMode) return g_outClient.connected() ? 1 : 0;
  uint32_t now = millis();
  uint16_t n = 0;
  for (int i = 0; i < MB_TRACK_MAX; i++) {
    if (g_track[i].used && (now - g_track[i].lastSeenMs) < MB_TRACK_WINDOW_MS) n++;
  }
  return n;
}

void mbTcpServerInit() {
  if (mbTcpClientMode) {
    // Bound how long a stalled/unreachable remote can block loop() for —
    // default library timeout is 10s, which would freeze RTU polling and
    // the web UI too since everything shares this one thread.
    g_outClient.setConnectionTimeout(2000);
    addLog("[MBTCP] client mode -> will dial " + mbTcpClientHost + ":" + String(mbTcpClientPort));
  } else {
    mbServer = new EthernetServer(modbusTcpPort);
    mbServer->begin();
    addLog("[MBTCP] server started on port " + String(modbusTcpPort));
  }
}

// ================================================================
//  Address resolution helpers
// ================================================================
static int findExact(uint8_t unitId, uint8_t regType, uint16_t addr) {
  for (uint8_t i = 0; i < MAX_POINTS; i++) {
    PointConfig &p = points[i];
    if (!p.enable) continue;
    if (p.tcpUnitId != unitId || p.tcpRegType != regType) continue;
    if (p.tcpAddr == addr) return i;
  }
  return -1;
}

// For register reads: a 2-register point also answers at tcpAddr+1 (word 1).
static bool findRegisterWord(uint8_t unitId, uint8_t regType, uint16_t addr, int &idx, uint8_t &wordIdx) {
  for (uint8_t i = 0; i < MAX_POINTS; i++) {
    PointConfig &p = points[i];
    if (!p.enable) continue;
    if (p.tcpUnitId != unitId || p.tcpRegType != regType) continue;
    if (p.tcpAddr == addr) { idx = i; wordIdx = 0; return true; }
    if (formatRegCount(p.format) == 2 && (uint16_t)(p.tcpAddr + 1) == addr) { idx = i; wordIdx = 1; return true; }
  }
  return false;
}

static void buildException(uint8_t func, uint8_t code, uint8_t *out, size_t &outLen) {
  out[0] = func | 0x80;
  out[1] = code;
  outLen = 2;
}

// ================================================================
//  PDU handling
// ================================================================
static void handleReadBits(uint8_t unitId, uint8_t func, uint8_t regType,
                             const uint8_t *pdu, size_t pduLen, uint8_t *out, size_t &outLen) {
  if (pduLen < 5) { buildException(func, MB_EXC_ILLEGAL_VALUE, out, outLen); return; }
  uint16_t startAddr = ((uint16_t)pdu[1] << 8) | pdu[2];
  uint16_t qty       = ((uint16_t)pdu[3] << 8) | pdu[4];
  if (qty == 0 || qty > MB_TCP_MAX_QTY_BITS) { buildException(func, MB_EXC_ILLEGAL_VALUE, out, outLen); return; }

  uint8_t bits[MB_TCP_MAX_QTY_BITS];
  for (uint16_t i = 0; i < qty; i++) {
    int idx = findExact(unitId, regType, (uint16_t)(startAddr + i));
    if (idx < 0) { buildException(func, MB_EXC_ILLEGAL_ADDRESS, out, outLen); return; }
    bits[i] = pointState[idx].rawRegs[0] ? 1 : 0;
  }

  uint8_t byteCount = (uint8_t)((qty + 7) / 8);
  out[0] = func;
  out[1] = byteCount;
  memset(&out[2], 0, byteCount);
  for (uint16_t i = 0; i < qty; i++) if (bits[i]) out[2 + i / 8] |= (uint8_t)(1 << (i % 8));
  outLen = 2 + byteCount;
}

static void handleReadRegs(uint8_t unitId, uint8_t func, uint8_t regType,
                            const uint8_t *pdu, size_t pduLen, uint8_t *out, size_t &outLen) {
  if (pduLen < 5) { buildException(func, MB_EXC_ILLEGAL_VALUE, out, outLen); return; }
  uint16_t startAddr = ((uint16_t)pdu[1] << 8) | pdu[2];
  uint16_t qty       = ((uint16_t)pdu[3] << 8) | pdu[4];
  if (qty == 0 || qty > MB_TCP_MAX_QTY_REGS) { buildException(func, MB_EXC_ILLEGAL_VALUE, out, outLen); return; }

  out[0] = func;
  out[1] = (uint8_t)(qty * 2);
  for (uint16_t i = 0; i < qty; i++) {
    int idx; uint8_t wordIdx;
    if (!findRegisterWord(unitId, regType, (uint16_t)(startAddr + i), idx, wordIdx)) {
      buildException(func, MB_EXC_ILLEGAL_ADDRESS, out, outLen);
      return;
    }
    uint16_t regVal = pointState[idx].rawRegs[wordIdx];
    out[2 + i * 2]     = (uint8_t)(regVal >> 8);
    out[2 + i * 2 + 1] = (uint8_t)(regVal & 0xFF);
  }
  outLen = 2 + (size_t)qty * 2;
}

static void handleWriteSingleCoil(uint8_t unitId, const uint8_t *pdu, size_t pduLen, uint8_t *out, size_t &outLen) {
  if (pduLen < 5) { buildException(0x05, MB_EXC_ILLEGAL_VALUE, out, outLen); return; }
  uint16_t addr = ((uint16_t)pdu[1] << 8) | pdu[2];
  uint16_t val  = ((uint16_t)pdu[3] << 8) | pdu[4];
  if (val != 0x0000 && val != 0xFF00) { buildException(0x05, MB_EXC_ILLEGAL_VALUE, out, outLen); return; }

  int idx = findExact(unitId, TCP_COIL, addr);
  if (idx < 0) {
    addLog("[TCPWR] FC05 unit=" + String(unitId) + " addr=" + String(addr) + " -> no matching point");
    buildException(0x05, MB_EXC_ILLEGAL_ADDRESS, out, outLen);
    return;
  }
  if (!points[idx].writable) {
    addLog(String("[TCPWR] ") + points[idx].name + " FC05 -> rejected, not writable");
    buildException(0x05, MB_EXC_ILLEGAL_FUNCTION, out, outLen);
    return;
  }

  uint16_t raw[2] = { (uint16_t)(val == 0xFF00 ? 1 : 0), 0 };
  unsigned long tW = millis();
  bool wOk = mbRtuWriteRaw(points[idx], pointState[idx], raw);
  unsigned long wElapsed = millis() - tW;
  if (!wOk) {
    addLog(String("[TCPWR] ") + points[idx].name + " FC05 val=" + String(raw[0]) +
           " -> RTU write failed: " + mbRtuFailReasonText(mbRtuLastFailReason()) +
           " (" + wElapsed + "ms)");
    buildException(0x05, MB_EXC_SERVER_FAILURE, out, outLen);
    return;
  }
  addLog(String("[TCPWR] ") + points[idx].name + " FC05 val=" + String(raw[0]) + " -> OK (" + wElapsed + "ms)");
  memcpy(out, pdu, 5);
  outLen = 5;
}

static void handleWriteSingleReg(uint8_t unitId, const uint8_t *pdu, size_t pduLen, uint8_t *out, size_t &outLen) {
  if (pduLen < 5) { buildException(0x06, MB_EXC_ILLEGAL_VALUE, out, outLen); return; }
  uint16_t addr = ((uint16_t)pdu[1] << 8) | pdu[2];
  uint16_t val  = ((uint16_t)pdu[3] << 8) | pdu[4];

  int idx = findExact(unitId, TCP_HOLDING, addr);
  if (idx < 0 || formatRegCount(points[idx].format) != 1) {
    addLog("[TCPWR] FC06 unit=" + String(unitId) + " addr=" + String(addr) + " -> no matching 1-register point");
    buildException(0x06, MB_EXC_ILLEGAL_ADDRESS, out, outLen);
    return;
  }
  if (!points[idx].writable) {
    addLog(String("[TCPWR] ") + points[idx].name + " FC06 -> rejected, not writable");
    buildException(0x06, MB_EXC_ILLEGAL_FUNCTION, out, outLen);
    return;
  }

  uint16_t raw[2] = { val, 0 };
  unsigned long tW = millis();
  bool wOk = mbRtuWriteRaw(points[idx], pointState[idx], raw);
  unsigned long wElapsed = millis() - tW;
  if (!wOk) {
    addLog(String("[TCPWR] ") + points[idx].name + " FC06 val=" + String(val) +
           " -> RTU write failed: " + mbRtuFailReasonText(mbRtuLastFailReason()) +
           " (" + wElapsed + "ms)");
    buildException(0x06, MB_EXC_SERVER_FAILURE, out, outLen);
    return;
  }
  addLog(String("[TCPWR] ") + points[idx].name + " FC06 val=" + String(val) + " -> OK (" + wElapsed + "ms)");
  memcpy(out, pdu, 5);
  outLen = 5;
}

static void handleWriteMultiReg(uint8_t unitId, const uint8_t *pdu, size_t pduLen, uint8_t *out, size_t &outLen) {
  if (pduLen < 6) { buildException(0x10, MB_EXC_ILLEGAL_VALUE, out, outLen); return; }
  uint16_t addr      = ((uint16_t)pdu[1] << 8) | pdu[2];
  uint16_t qty       = ((uint16_t)pdu[3] << 8) | pdu[4];
  uint8_t  byteCount = pdu[5];

  if (byteCount != qty * 2 || pduLen != (size_t)(6 + byteCount) || (qty != 1 && qty != 2)) {
    buildException(0x10, MB_EXC_ILLEGAL_VALUE, out, outLen);
    return;
  }

  int idx = findExact(unitId, TCP_HOLDING, addr);
  if (idx < 0 || formatRegCount(points[idx].format) != qty) {
    addLog("[TCPWR] FC16 unit=" + String(unitId) + " addr=" + String(addr) + " qty=" + String(qty) +
           " -> no matching point (or wrong register count)");
    buildException(0x10, MB_EXC_ILLEGAL_ADDRESS, out, outLen);
    return;
  }
  if (!points[idx].writable) {
    addLog(String("[TCPWR] ") + points[idx].name + " FC16 -> rejected, not writable");
    buildException(0x10, MB_EXC_ILLEGAL_FUNCTION, out, outLen);
    return;
  }

  uint16_t raw[2] = { 0, 0 };
  raw[0] = ((uint16_t)pdu[6] << 8) | pdu[7];
  if (qty == 2) raw[1] = ((uint16_t)pdu[8] << 8) | pdu[9];

  unsigned long tW = millis();
  bool wOk = mbRtuWriteRaw(points[idx], pointState[idx], raw);
  unsigned long wElapsed = millis() - tW;
  if (!wOk) {
    addLog(String("[TCPWR] ") + points[idx].name + " FC16 -> RTU write failed: " +
           mbRtuFailReasonText(mbRtuLastFailReason()) + " (" + wElapsed + "ms)");
    buildException(0x10, MB_EXC_SERVER_FAILURE, out, outLen);
    return;
  }
  addLog(String("[TCPWR] ") + points[idx].name + " FC16 -> OK (" + wElapsed + "ms)");
  memcpy(out, pdu, 5);   // func + addr + qty
  outLen = 5;
}

static void buildResponse(uint8_t unitId, const uint8_t *pdu, size_t pduLen, uint8_t *out, size_t &outLen) {
  uint8_t func = pdu[0];
  switch (func) {
    case 0x01: handleReadBits(unitId, func, TCP_COIL,     pdu, pduLen, out, outLen); break;
    case 0x02: handleReadBits(unitId, func, TCP_DISCRETE, pdu, pduLen, out, outLen); break;
    case 0x03: handleReadRegs(unitId, func, TCP_HOLDING,  pdu, pduLen, out, outLen); break;
    case 0x04: handleReadRegs(unitId, func, TCP_INPUT,    pdu, pduLen, out, outLen); break;
    case 0x05: handleWriteSingleCoil(unitId, pdu, pduLen, out, outLen); break;
    case 0x06: handleWriteSingleReg(unitId, pdu, pduLen, out, outLen);  break;
    case 0x10: handleWriteMultiReg(unitId, pdu, pduLen, out, outLen);   break;
    default:   buildException(func, MB_EXC_ILLEGAL_FUNCTION, out, outLen); break;
  }
}

static void sendMbapResponse(EthernetClient &c, uint8_t transIdHi, uint8_t transIdLo, uint8_t unitId,
                              const uint8_t *respPdu, size_t respLen) {
  uint8_t frame[7 + 260];
  frame[0] = transIdHi; frame[1] = transIdLo;
  frame[2] = 0;         frame[3] = 0;        // protocol id = 0
  uint16_t outLen = (uint16_t)(1 + respLen);
  frame[4] = (uint8_t)(outLen >> 8);
  frame[5] = (uint8_t)(outLen & 0xFF);
  frame[6] = unitId;
  memcpy(&frame[7], respPdu, respLen);
  c.write(frame, 7 + respLen);
}

// ================================================================
//  Write requests (FC05/06/16) that land while an RTU read is already in
//  flight can't be serviced on the spot — see mbRtuIsBusy(). Rather than
//  instantly failing them (which is what a plain g_rtuBusy check on the
//  RTU side does), hold the single most recent one here and run it the
//  moment the bus frees up, instead of making the TCP client eat a bogus
//  "device failure" for a request we never even attempted.
// ================================================================
struct PendingWrite {
  bool          active = false;
  EthernetClient client;
  uint8_t       transIdHi, transIdLo, unitId;
  uint8_t       pdu[252];
  size_t        pduLen;
};
static PendingWrite g_pendingWrite;

static void runPendingWriteIfReady() {
  if (!g_pendingWrite.active || mbRtuIsBusy()) return;

  PendingWrite pw = g_pendingWrite;   // copy out, then clear, before we (re)enter RTU code
  g_pendingWrite.active = false;

  if (!pw.client.connected()) {
    addLog("[TCPWR] queued write dropped — client disconnected before bus freed up");
    return;
  }

  uint8_t respPdu[260];
  size_t  respLen = 0;
  buildResponse(pw.unitId, pw.pdu, pw.pduLen, respPdu, respLen);
  sendMbapResponse(pw.client, pw.transIdHi, pw.transIdLo, pw.unitId, respPdu, respLen);
}

// ================================================================
//  serviceClient – reads one MBAP+PDU request off c (if any) and answers
//  it. Shared by both listen mode (per accepted client) and dial-out mode
//  (the single persistent outbound connection) — the Modbus-level handling
//  is identical either way, only how "c" was obtained differs.
// ================================================================
static void serviceClient(EthernetClient &c) {
  if (!c.available()) return;   // nothing pending on this connection right now

  trackClient(c);

  uint8_t hdr[7];
  size_t got = 0;
  unsigned long t0 = millis();
  while (got < 7 && millis() - t0 < 200) {
    if (c.available()) hdr[got++] = (uint8_t)c.read();
  }
  if (got < 7) return;   // partial/garbage frame — drop, client will retry

  uint16_t protoId = ((uint16_t)hdr[2] << 8) | hdr[3];
  uint16_t length  = ((uint16_t)hdr[4] << 8) | hdr[5];
  uint8_t  unitId  = hdr[6];

  if (protoId != 0 || length < 2 || length > 253) return;   // not Modbus TCP

  uint8_t pdu[252];
  size_t pduLen = (size_t)length - 1;   // length includes unitId, already consumed
  if (pduLen == 0 || pduLen > sizeof(pdu)) return;

  size_t gotPdu = 0;
  t0 = millis();
  while (gotPdu < pduLen && millis() - t0 < 200) {
    if (c.available()) pdu[gotPdu++] = (uint8_t)c.read();
  }
  if (gotPdu < pduLen) return;

  uint8_t func = pdu[0];
  bool isWriteFunc = (func == 0x05 || func == 0x06 || func == 0x10);

  if (isWriteFunc && mbRtuIsBusy()) {
    if (g_pendingWrite.active) {
      // Already holding one queued write — a second collision is rare
      // enough (RS485 transactions run ~200ms-2s) that failing it outright
      // is fine rather than adding a second queue slot.
      uint8_t respPdu[2]; size_t respLen;
      buildException(func, MB_EXC_SERVER_FAILURE, respPdu, respLen);
      sendMbapResponse(c, hdr[0], hdr[1], unitId, respPdu, respLen);
      addLog("[TCPWR] write rejected — a write was already queued waiting for the bus");
      return;
    }
    g_pendingWrite.active    = true;
    g_pendingWrite.client    = c;
    g_pendingWrite.transIdHi = hdr[0];
    g_pendingWrite.transIdLo = hdr[1];
    g_pendingWrite.unitId    = unitId;
    memcpy(g_pendingWrite.pdu, pdu, pduLen);
    g_pendingWrite.pduLen    = pduLen;
    addLog("[TCPWR] write queued — bus busy with a read, will run as soon as it's free");
    return;   // no response yet; runPendingWriteIfReady() sends it once the bus frees up
  }

  uint8_t respPdu[260];
  size_t  respLen = 0;
  buildResponse(unitId, pdu, pduLen, respPdu, respLen);
  sendMbapResponse(c, hdr[0], hdr[1], unitId, respPdu, respLen);
}

// ================================================================
//  Dial-out (Client mode): keep one outbound connection alive, retrying
//  on a fixed interval rather than every loop() iteration.
// ================================================================
static void serviceDialOut() {
  if (!g_outClient.connected()) {
    unsigned long now = millis();
    if (g_lastConnectAttemptMs != 0 && now - g_lastConnectAttemptMs < MB_RECONNECT_INTERVAL_MS) return;
    g_lastConnectAttemptMs = now;
    g_outClient.stop();
    addLog("[MBTCP] connecting to " + mbTcpClientHost + ":" + String(mbTcpClientPort) + " ...");
    bool ok = g_outClient.connect(mbTcpClientHost.c_str(), (uint16_t)mbTcpClientPort);
    addLog(ok ? "[MBTCP] connected" : "[MBTCP] connect failed, will retry");
    return;
  }
  serviceClient(g_outClient);
}

// ================================================================
//  mbTcpServerLoop – services exactly one ready client per call
// ================================================================
void mbTcpServerLoop() {
  runPendingWriteIfReady();

  if (mbTcpClientMode) {
    serviceDialOut();
    return;
  }

  EthernetClient c = mbServer->available();
  if (!c) return;
  serviceClient(c);
}
