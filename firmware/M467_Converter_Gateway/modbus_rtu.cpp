// ================================================================
//  modbus_rtu.cpp  –  Modbus RTU master (RS485)
//
//  Send/receive timing (RS485.noReceive()->beginTransmission->write->
//  endTransmission->receive(), timeout+CRC check) follows the same
//  pattern already proven on this exact board/library stack in the
//  reference Gateway Manager firmware (modbusRtuReadRegs / modbusRtuWriteReg
//  in kernel.cpp). This product has no points table — every request is a
//  live pass-through transaction (see mbRtuRawTransaction/
//  mbRtuRawBytesTransaction below), unlike the sibling Concentrator
//  product which polls a fixed table on its own schedule.
// ================================================================
#include "modbus_rtu.h"
#include "board.h"
#include <math.h>

// ================================================================
//  HardwareSerial::flush() on this Arduino core is a no-op (see
//  cores/nuvoton/HardwareSerial.cpp — "No need to implement because we
//  use hardware UART and it is with hardware FIFO", empty body). That
//  means RS485.endTransmission() — which calls flush() then immediately
//  drops the DE pin — can release the RS485 driver before the last byte
//  has actually finished shifting out on the wire, intermittently
//  truncating the CRC and causing the slave to silently drop the frame
//  (bad CRC => no response => our timeout).
//
//  Originally this polled the UART's FIFOSTS.TXEMPTYF hardware flag, but on
//  this board that flag never set within any reasonable window (confirmed
//  empirically — it hit its safety cap on every single transaction), so
//  it was just quietly falling back to a fixed delay anyway. A later attempt
//  to replace it with delayMicroseconds() locked the whole board up solid
//  (no serial output, no web/TCP response at all) — delayMicroseconds()'s
//  actual implementation for this core isn't in any source file in the
//  board package (likely buried in the prebuilt libchip_M460_gcc_rel.a),
//  so its correctness here is unknown/unverified. micros()-based busy
//  waiting, by contrast, has been empirically proven reliable on this exact
//  board across many earlier test runs (it's what drove the 5ms safety cap
//  above, consistently, every time) — so use only that, nothing else.
// ================================================================
static uint32_t charTimeUs() {
  uint32_t baud = baudFromIndex(S0.baudIndex);
  uint32_t bitsPerChar = 1 + S0.dataBits + (S0.parity != 0 ? 1 : 0) + S0.stopBits;   // start+data+parity+stop
  return (bitsPerChar * 1000000UL) / baud;
}

static void waitTxReallyDone() {
  uint32_t waitUs = charTimeUs() * 2;   // 2 character times of margin
  unsigned long t0 = micros();
  while (micros() - t0 < waitUs) { /* busy wait — see note above */ }
}

// Standard Modbus RTU silent-interval rule for delimiting a frame without a
// length field: ~3.5 character times, except the spec pins this to a fixed
// 1750us above 19200 baud (3.5 chars would otherwise get impractically
// tight at high speed). Used by readUntilSilence() below. Not static:
// modbus_converter.cpp's transparent sub-mode reuses this same gap on the
// TCP side too, since a transparent client is expected to pace its bytes as
// if writing straight onto the RTU wire at S0's baud rate.
uint32_t interFrameSilenceUs() {
  uint32_t baud = baudFromIndex(S0.baudIndex);
  if (baud > 19200) return 1750UL;
  return charTimeUs() * 4;   // ~3.5 chars, rounded up for margin against our own loop jitter
}

// ================================================================
//  Diagnostics: record *why* the last RTU transaction failed — distinguishing
//  a bare timeout (nothing came back) from a real Modbus exception (slave
//  replied "no" for a specific reason) from a corrupted/garbled response
//  (CRC or length mismatch).
// ================================================================
static uint8_t g_lastFailReason        = MBRTU_OK;
static uint8_t g_lastFailLen           = 0;
static uint8_t g_lastFailExceptionCode = 0;

uint8_t mbRtuLastFailReason()        { return g_lastFailReason; }
uint8_t mbRtuLastFailLen()           { return g_lastFailLen; }
uint8_t mbRtuLastFailExceptionCode() { return g_lastFailExceptionCode; }

// ================================================================
//  RS485 is a single half-duplex bus — only one transaction can be in
//  flight at a time. Since the wait loops below call mbConverterLoop() to
//  avoid starving other TCP clients, a second request arriving during that
//  call would otherwise try to start a second RTU transaction while the
//  first one is still waiting for its response, corrupting both. Guard
//  with a simple busy flag — mbConverterLoop() defers a colliding turn
//  rather than starting it, see modbus_converter.cpp.
// ================================================================
static volatile bool g_rtuBusy = false;
struct RtuBusyGuard {
  RtuBusyGuard()  { g_rtuBusy = true; }
  ~RtuBusyGuard() { g_rtuBusy = false; }
};

bool mbRtuIsBusy() { return g_rtuBusy; }

String mbRtuFailReasonText(uint8_t reason) {
  switch (reason) {
    case MBRTU_TIMEOUT:    return "timeout, no/short response";
    case MBRTU_EXCEPTION:  return "slave exception 0x" + String(g_lastFailExceptionCode, HEX);
    case MBRTU_BAD_HEADER: return "unexpected slave/func in response";
    case MBRTU_BAD_CRC:    return "CRC mismatch";
    case MBRTU_BAD_LEN:    return "byte count mismatch";
    case MBRTU_BUS_BUSY:   return "bus busy — rejected instantly, another transaction was in flight";
    default:                return "ok";
  }
}

// ================================================================
//  CRC16 (Modbus polynomial 0xA001)
// ================================================================
static uint16_t modbusCRC16(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; b++) {
      if (crc & 1) crc = (crc >> 1) ^ 0xA001;
      else         crc >>= 1;
    }
  }
  return crc;
}

// ================================================================
//  Serial line setup
//
//  RS485.begin(baud) only ever programs 8-N-1 on this Arduino core
//  (NuMicro UART_Open() hardcodes UART_WORD_LEN_8|UART_PARITY_NONE|
//  UART_STOP_BIT_1 and ignores the "config" argument other than to flag
//  RS485-AUD mode). To actually honor the configured data bits / parity /
//  stop bits we re-apply them with the NuMicro BSP's UART_SetLineConfig()
//  right after RS485.begin() sets up the ISR/ring buffer and RS485
//  auto-direction hardware mode. UART_Desc[1] is the descriptor for
//  Serial1, which is the port ArduinoRS485 uses on this board (see
//  RS485_SERIAL_PORT in ArduinoRS485's RS485.h).
// ================================================================
static uint32_t wordLenCode(uint8_t bits) {
  switch (bits) {
    case 5: return UART_WORD_LEN_5;
    case 6: return UART_WORD_LEN_6;
    case 7: return UART_WORD_LEN_7;
    default: return UART_WORD_LEN_8;
  }
}

static uint32_t parityCode(uint8_t parity) {
  switch (parity) {
    case 1: return UART_PARITY_ODD;
    case 2: return UART_PARITY_EVEN;
    default: return UART_PARITY_NONE;
  }
}

static uint32_t stopBitCode(uint8_t stopBits) {
  return (stopBits >= 2) ? UART_STOP_BIT_2 : UART_STOP_BIT_1;
}

void mbRtuBegin() {
  uint32_t baud = baudFromIndex(S0.baudIndex);
  // Must pass RS485_OVER_SERIAL (not 0) so the core still arms the UART's
  // RS485 auto-direction hardware mode the same way RS485.begin(baud) does.
  RS485.begin(baud, (uint16_t)RS485_OVER_SERIAL, (int)S0.preDelayUs, (int)S0.postDelayUs);

  UART_SetLineConfig(UART_Desc[1].U, baud,
                      wordLenCode(S0.dataBits),
                      parityCode(S0.parity),
                      stopBitCode(S0.stopBits));

  addLog("[RTU] RS485 begin baud=" + String(baud) +
         " bits=" + String(S0.dataBits) +
         " parity=" + String(S0.parity) +
         " stop=" + String(S0.stopBits));
}

// ================================================================
//  Generic pass-through primitives — we don't know the expected response
//  length in advance (the client can ask for anything), so frames are
//  delimited by the RS485 silent-interval rule instead of a byte count.
// ================================================================

// Reads from s (RS485 or a TCP client, both are Stream) until either
// nothing has arrived within overallTimeoutMs, or at least one byte has
// arrived and then gapUs of silence follows (frame considered complete).
// Interleaves converter/web/LED servicing + WDT feed during the wait so a
// slow/absent RTU slave (or a slow TCP client, in transparent mode) can't
// starve everything else sharing this thread. Not static: shared with
// modbus_converter.cpp's transparent sub-mode, which uses it on the TCP
// client stream to find where one client "frame" ends.
size_t readUntilSilence(Stream &s, uint8_t *buf, size_t maxLen,
                         unsigned long overallTimeoutMs, uint32_t gapUs) {
  size_t len = 0;
  bool   gotAny = false;
  unsigned long t0 = millis();
  unsigned long lastByteUs = micros();

  while (millis() - t0 < overallTimeoutMs) {
    if (len < maxLen && s.available()) {
      buf[len++] = (uint8_t)s.read();
      lastByteUs = micros();
      gotAny = true;
      continue;   // drain a burst fast, don't waste time re-checking the gap mid-frame
    }
    if (gotAny && (micros() - lastByteUs) > gapUs) break;   // frame complete
    if (len >= maxLen) break;
    WDT_RESET_COUNTER();
    WDT_MARK("rtu:silence:converter");  mbConverterLoop();
    WDT_MARK("rtu:silence:frontend");   frontendLoop();
    WDT_MARK("rtu:silence:led");        boardLedService();
  }
  return len;
}

// Structured passthrough: we own the CRC (add on TX, verify on RX) and hand
// the caller back just the PDU, matching how a normal RTU master frame is
// built. A genuine slave exception is treated as a *successful* transaction
// (the slave did answer) — the 2-byte exception PDU is returned as-is so
// the caller (the converter) can relay it verbatim, which is the standard,
// expected behavior of a protocol gateway.
bool mbRtuRawTransaction(uint8_t slaveId, const uint8_t *reqPdu, size_t reqPduLen,
                          uint8_t *respPdu, size_t &respPduLen, size_t maxRespPduLen) {
  if (g_rtuBusy) { g_lastFailReason = MBRTU_BUS_BUSY; g_lastFailLen = 0; return false; }
  RtuBusyGuard busyGuard;

  if (reqPduLen == 0 || reqPduLen > 250) { g_lastFailReason = MBRTU_BAD_LEN; return false; }

  uint8_t tx[256];
  tx[0] = slaveId;
  memcpy(&tx[1], reqPdu, reqPduLen);
  uint16_t crc = modbusCRC16(tx, 1 + reqPduLen);
  tx[1 + reqPduLen]     = (uint8_t)(crc & 0xFF);
  tx[1 + reqPduLen + 1] = (uint8_t)(crc >> 8);
  size_t txLen = 1 + reqPduLen + 2;

  while (RS485.available()) (void)RS485.read();
  RS485.noReceive();
  RS485.beginTransmission();
  RS485.write(tx, txLen);
  waitTxReallyDone();
  RS485.endTransmission();
  RS485.receive();

  uint8_t rx[256];
  size_t rxLen = readUntilSilence(RS485, rx, sizeof(rx), S0.responseTimeoutMs, interFrameSilenceUs());
  g_lastFailLen = (uint8_t)rxLen;

  if (rxLen == 5 && (rx[1] & 0x80)) {
    uint16_t excCrc = (uint16_t)rx[3] | ((uint16_t)rx[4] << 8);
    if (rx[0] == slaveId && excCrc == modbusCRC16(rx, 3)) {
      g_lastFailReason        = MBRTU_EXCEPTION;
      g_lastFailExceptionCode = rx[2];
      if (2 > maxRespPduLen) return false;
      respPdu[0] = rx[1];
      respPdu[1] = rx[2];
      respPduLen = 2;
      return true;   // relay the real exception verbatim, see comment above
    }
  }

  if (rxLen < 4)          { g_lastFailReason = MBRTU_TIMEOUT;    return false; }   // slaveId+func+CRC16 minimum
  if (rx[0] != slaveId)   { g_lastFailReason = MBRTU_BAD_HEADER; return false; }

  uint16_t rxCrc = (uint16_t)rx[rxLen - 2] | ((uint16_t)rx[rxLen - 1] << 8);
  if (rxCrc != modbusCRC16(rx, rxLen - 2)) { g_lastFailReason = MBRTU_BAD_CRC; return false; }

  size_t pduLen = rxLen - 1 - 2;   // minus slaveId, minus CRC
  if (pduLen == 0 || pduLen > maxRespPduLen) { g_lastFailReason = MBRTU_BAD_LEN; return false; }
  memcpy(respPdu, &rx[1], pduLen);
  respPduLen = pduLen;
  return true;
}

// Fully raw: the caller already built slaveId+PDU+CRC themselves (a
// transparent-mode TCP client's own bytes) — forwarded byte-for-byte, no
// interpretation, no CRC check on either side. The response is likewise
// handed back exactly as received; it's the TCP client's job to validate it.
bool mbRtuRawBytesTransaction(const uint8_t *rawTx, size_t rawTxLen,
                               uint8_t *rawRx, size_t &rawRxLen, size_t maxRawRxLen) {
  if (g_rtuBusy) { g_lastFailReason = MBRTU_BUS_BUSY; g_lastFailLen = 0; return false; }
  RtuBusyGuard busyGuard;

  if (rawTxLen == 0) { g_lastFailReason = MBRTU_BAD_LEN; return false; }

  while (RS485.available()) (void)RS485.read();
  RS485.noReceive();
  RS485.beginTransmission();
  RS485.write(rawTx, rawTxLen);
  waitTxReallyDone();
  RS485.endTransmission();
  RS485.receive();

  size_t rxLen = readUntilSilence(RS485, rawRx, maxRawRxLen, S0.responseTimeoutMs, interFrameSilenceUs());
  g_lastFailLen = (uint8_t)rxLen;
  rawRxLen = rxLen;

  if (rxLen == 0) { g_lastFailReason = MBRTU_TIMEOUT; return false; }
  g_lastFailReason = MBRTU_OK;
  return true;
}
