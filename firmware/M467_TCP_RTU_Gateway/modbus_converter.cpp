// ================================================================
//  modbus_converter.cpp – TCP<->RTU converter mode (GW_CONVERTER)
//
//  Unlike the concentrator, every request here needs a live RS485
//  transaction (no points cache, no address translation) — so fairness
//  among simultaneous TCP clients means fairness for *turns at the shared
//  RS485 bus*, not just CPU time. mbConverterLoop() dispatches at most one
//  client per call, in round-robin order over our own tracked-client list
//  (same fairness rationale as modbus_tcp_server.cpp's rrTrack/rrPrune —
//  EthernetServer::available() always scans from socket index 0 and would
//  let one busy client starve everyone accepted after it), and defers
//  entirely (no queueing) if the bus is already busy with someone else's
//  transaction — see mbRtuIsBusy(). Deferring rather than partially reading
//  a request and queuing it keeps this module simple: a dispatched client
//  either completes a full request/response round trip, or nothing happens
//  for it this call and it's picked up again the next.
//
//  CONV_STANDARD: real Modbus TCP<->RTU protocol conversion. We parse the
//  MBAP header + PDU, forward the PDU over RS485 via mbRtuRawTransaction()
//  (which adds/verifies the CRC for us), and translate an RTU-side failure
//  into a Modbus exception code the TCP client can understand. No points
//  table, no address translation, no capacity limit — the PDU goes through
//  as-is for every function code.
//
//  CONV_TRANSPARENT: raw byte tunnel — the TCP payload the client sends IS
//  the RTU frame content verbatim (client computes its own CRC). Since
//  there's no length field to know where one client's frame ends, we reuse
//  the RS485 silent-interval rule (readUntilSilence/interFrameSilenceUs,
//  shared with modbus_rtu.cpp) on the TCP stream too — a transparent client
//  is expected to pace its bytes as if writing straight onto the RTU wire.
// ================================================================
#include "modbus_converter.h"
#include "board.h"

static EthernetServer *g_convServer = nullptr;

// ---- our own tracked-client list for round-robin fairness ----
struct ConvClient { bool used; bool busy; EthernetClient client; uint32_t ip; uint16_t port; };
static ConvClient g_conv[MAX_TRACKED_CLIENTS];
static uint8_t    g_convNext = 0;

static void convTrack(EthernetClient &c) {
  uint32_t ip   = (uint32_t)c.remoteIP();
  uint16_t port = c.remotePort();
  for (uint8_t i = 0; i < MAX_TRACKED_CLIENTS; i++) {
    if (g_conv[i].used && g_conv[i].ip == ip && g_conv[i].port == port) return;   // already tracked
  }
  for (uint8_t i = 0; i < MAX_TRACKED_CLIENTS; i++) {
    if (!g_conv[i].used) { g_conv[i] = { true, false, c, ip, port }; return; }
  }
  // Tracked table full — this client won't get a round-robin turn until a slot frees up.
}

static void convPrune() {
  for (uint8_t i = 0; i < MAX_TRACKED_CLIENTS; i++) {
    if (g_conv[i].used && !g_conv[i].busy && !g_conv[i].client.connected()) g_conv[i] = ConvClient();
  }
}

uint16_t mbConverterActiveClientCount() {
  uint16_t n = 0;
  for (uint8_t i = 0; i < MAX_TRACKED_CLIENTS; i++) {
    if (g_conv[i].used && g_conv[i].client.connected()) n++;
  }
  return n;
}

void mbConverterInit() {
  if (gatewayMode != GW_CONVERTER) return;
  g_convServer = new EthernetServer(modbusTcpPort);
  g_convServer->begin();
  addLog(String("[CONV] TCP<->RTU converter started on port ") + modbusTcpPort +
         " (" + (converterMode == CONV_TRANSPARENT ? "transparent" : "standard") + ")");
}

// ================================================================
//  CONV_STANDARD — real Modbus TCP<->RTU conversion
// ================================================================
static void sendConvMbapResponse(EthernetClient &c, uint8_t transIdHi, uint8_t transIdLo, uint8_t unitId,
                                  const uint8_t *respPdu, size_t respLen) {
  uint8_t frame[7 + 260];
  frame[0] = transIdHi; frame[1] = transIdLo;
  frame[2] = 0;         frame[3] = 0;   // protocol id = 0
  uint16_t outLen = (uint16_t)(1 + respLen);
  frame[4] = (uint8_t)(outLen >> 8);
  frame[5] = (uint8_t)(outLen & 0xFF);
  frame[6] = unitId;
  memcpy(&frame[7], respPdu, respLen);
  c.write(frame, 7 + respLen);
}

static uint8_t rtuFailToMbException(uint8_t reason) {
  switch (reason) {
    case MBRTU_BUS_BUSY: return MB_CONV_EXC_GATEWAY_BUSY;
    case MBRTU_TIMEOUT:  return MB_CONV_EXC_GATEWAY_NO_RESPONSE;
    default:             return MB_CONV_EXC_SERVER_FAILURE;   // bad header/CRC/length from the device
  }
}

static void serviceConverterStandard(EthernetClient &c) {
  uint8_t hdr[7];
  size_t got = 0;
  unsigned long t0 = millis();
  while (got < 7 && millis() - t0 < CONV_MBAP_CHUNK_TIMEOUT_MS) {
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
  while (gotPdu < pduLen && millis() - t0 < CONV_MBAP_CHUNK_TIMEOUT_MS) {
    if (c.available()) pdu[gotPdu++] = (uint8_t)c.read();
  }
  if (gotPdu < pduLen) return;

  uint8_t respPdu[260];
  size_t  respLen = 0;
  if (!mbRtuRawTransaction(unitId, pdu, pduLen, respPdu, respLen, sizeof(respPdu))) {
    uint8_t exc[2] = { (uint8_t)(pdu[0] | 0x80), rtuFailToMbException(mbRtuLastFailReason()) };
    sendConvMbapResponse(c, hdr[0], hdr[1], unitId, exc, 2);
    return;
  }
  sendConvMbapResponse(c, hdr[0], hdr[1], unitId, respPdu, respLen);
}

// ================================================================
//  CONV_TRANSPARENT — raw byte tunnel
// ================================================================
static void serviceConverterTransparent(EthernetClient &c) {
  uint8_t rawTx[256];
  size_t rawTxLen = readUntilSilence(c, rawTx, sizeof(rawTx),
                                      CONV_CLIENT_FRAME_TIMEOUT_MS, interFrameSilenceUs());
  if (rawTxLen == 0) return;

  uint8_t rawRx[256];
  size_t  rawRxLen = 0;
  if (!mbRtuRawBytesTransaction(rawTx, rawTxLen, rawRx, rawRxLen, sizeof(rawRx))) {
    // No/garbled response from the RTU device — a raw tunnel has no framing
    // of its own to report that with, so send nothing back and let the
    // client's own timeout handle it, same as if it were wired to RS485
    // directly.
    return;
  }
  c.write(rawRx, rawRxLen);
}

// ================================================================
//  mbConverterLoop – services at most one ready tracked client per call, in
//  round-robin order, and only if the RS485 bus isn't already busy with
//  someone else's transaction. Self-guards on gatewayMode so it's safe to
//  call unconditionally from anywhere (e.g. the RTU wait loops).
// ================================================================
void mbConverterLoop() {
  if (gatewayMode != GW_CONVERTER || !g_convServer) return;

  // available() is used purely for discovering connections here — actual
  // dispatch below always goes through our own round-robin order, not scan
  // order. Safe to call even while the bus is busy (touches TCP only).
  EthernetClient discovered = g_convServer->available();
  if (discovered) convTrack(discovered);
  convPrune();

  if (mbRtuIsBusy()) return;   // someone else's transaction is already in flight — defer, don't queue

  for (uint8_t step = 0; step < MAX_TRACKED_CLIENTS; step++) {
    uint8_t i = (uint8_t)((g_convNext + step) % MAX_TRACKED_CLIENTS);
    ConvClient &slot = g_conv[i];
    if (!slot.used || slot.busy || !slot.client.connected() || !slot.client.available()) continue;

    // Mark busy for the whole turn, not just the RTU part — CONV_TRANSPARENT
    // reads the client's raw bytes over multiple iterations (readUntilSilence),
    // during which reentrant mbConverterLoop() calls (from the RTU wait
    // inside it, or a nested turn dispatched for a different client) must
    // not also pick this same client, or two loops would race reading the
    // same TCP stream.
    slot.busy = true;
    if (converterMode == CONV_TRANSPARENT) serviceConverterTransparent(slot.client);
    else                                    serviceConverterStandard(slot.client);
    slot.busy = false;

    g_convNext = (uint8_t)((i + 1) % MAX_TRACKED_CLIENTS);
    return;
  }
}
