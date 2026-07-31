// ================================================================
//  modbus_rtu.cpp  –  Modbus RTU master (RS485)
//
//  Send/receive timing (RS485.noReceive()->beginTransmission->write->
//  endTransmission->receive(), timeout+CRC check) follows the same
//  pattern already proven on this exact board/library stack in the
//  reference Gateway Manager firmware (modbusRtuReadRegs / modbusRtuWriteReg
//  in kernel.cpp). Only the point model and scheduling around it differ.
// ================================================================
#include "modbus_rtu.h"
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
static void waitTxReallyDone() {
  uint32_t baud = baudFromIndex(S0.baudIndex);
  uint32_t bitsPerChar = 1 + S0.dataBits + (S0.parity != 0 ? 1 : 0) + S0.stopBits;   // start+data+parity+stop
  uint32_t charTimeUs  = (bitsPerChar * 1000000UL) / baud;
  uint32_t waitUs = charTimeUs * 2;   // 2 character times of margin
  unsigned long t0 = micros();
  while (micros() - t0 < waitUs) { /* busy wait — see note above */ }
}

// ================================================================
//  Diagnostics: record *why* the last RTU transaction failed so callers
//  (poll_engine's addLog) can report something more useful than just
//  "read failed" — distinguishing a bare timeout (nothing came back) from
//  a real Modbus exception (slave replied "no" for a specific reason) from
//  a corrupted/garbled response (CRC or length mismatch).
// ================================================================
static uint8_t g_lastFailReason        = MBRTU_OK;
static uint8_t g_lastFailLen           = 0;
static uint8_t g_lastFailExceptionCode = 0;

uint8_t mbRtuLastFailReason()        { return g_lastFailReason; }
uint8_t mbRtuLastFailLen()           { return g_lastFailLen; }
uint8_t mbRtuLastFailExceptionCode() { return g_lastFailExceptionCode; }

// ================================================================
//  RS485 is a single half-duplex bus — only one transaction can be in
//  flight at a time. Since the wait loops below now call mbTcpServerLoop()
//  to avoid starving TCP clients, a *write* request arriving during that
//  call would otherwise try to start a second RTU transaction while the
//  first one is still waiting for its response, corrupting both. Guard
//  with a simple busy flag; a write that lands mid-transaction is rejected
//  immediately (surfaces as a Server Device Failure exception to the TCP
//  client, which can just retry) instead of touching the bus.
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
//  Low-level transactions. qty is always <=2 in this firmware (each
//  point is at most one 32-bit value / two registers, or one coil).
// ================================================================
static bool rtuReadRaw(uint8_t slaveId, uint8_t func, uint16_t addr, uint16_t qty,
                        uint8_t *outData, uint8_t &outLen) {
  if (g_rtuBusy) { g_lastFailReason = MBRTU_BUS_BUSY; g_lastFailLen = 0; return false; }
  RtuBusyGuard busyGuard;

  uint8_t tx[8];
  tx[0] = slaveId;
  tx[1] = func;
  tx[2] = (uint8_t)(addr >> 8);
  tx[3] = (uint8_t)(addr & 0xFF);
  tx[4] = (uint8_t)(qty >> 8);
  tx[5] = (uint8_t)(qty & 0xFF);
  uint16_t crc = modbusCRC16(tx, 6);
  tx[6] = (uint8_t)(crc & 0xFF);
  tx[7] = (uint8_t)(crc >> 8);

  while (RS485.available()) (void)RS485.read();   // purge stale bytes

  RS485.noReceive();
  RS485.beginTransmission();
  RS485.write(tx, 8);
  waitTxReallyDone();
  RS485.endTransmission();
  RS485.receive();

  bool isCoil = isCoilRtuFunc(func);
  uint8_t expDataBytes = isCoil ? (uint8_t)((qty + 7) / 8) : (uint8_t)(qty * 2);

  uint8_t rx[16];
  size_t  rxLen = 0;
  size_t  minLen = 5 + (size_t)expDataBytes;   // slaveId+func+byteCount+data+CRC(2)
  unsigned long t0 = millis();

  while (millis() - t0 < S0.responseTimeoutMs) {
    while (RS485.available() && rxLen < sizeof(rx)) rx[rxLen++] = (uint8_t)RS485.read();
    if (rxLen >= minLen) break;
    // A Modbus exception reply is always exactly 5 bytes (slaveId, func|0x80,
    // excCode, CRC lo, CRC hi) — shorter than the data response we're
    // actually waiting for here, so stop as soon as one looks complete
    // instead of burning the whole timeout on an answer we already have.
    if (rxLen >= 5 && (rx[1] & 0x80)) break;
    WDT_RESET_COUNTER();
    // Don't let a slow/absent RTU slave starve the Modbus TCP server for the
    // whole responseTimeoutMs window — service any pending TCP client on
    // every spin of this wait so the two paths don't fight for the CPU.
    mbTcpServerLoop();
  }
  g_lastFailLen = (uint8_t)rxLen;

  if (rxLen >= 5 && (rx[1] & 0x80)) {
    uint16_t excCrc = (uint16_t)rx[3] | ((uint16_t)rx[4] << 8);
    if (rx[0] == slaveId && rx[1] == (uint8_t)(func | 0x80) && excCrc == modbusCRC16(rx, 3)) {
      g_lastFailReason        = MBRTU_EXCEPTION;
      g_lastFailExceptionCode = rx[2];
      return false;
    }
    // Looked exception-shaped but didn't check out — fall through, the
    // generic checks below will report the more specific reason.
  }

  if (rxLen < minLen)                          { g_lastFailReason = MBRTU_TIMEOUT;    return false; }
  if (rx[0] != slaveId || rx[1] != func)       { g_lastFailReason = MBRTU_BAD_HEADER; return false; }

  uint16_t rxCrc = (uint16_t)rx[rxLen - 2] | ((uint16_t)rx[rxLen - 1] << 8);
  if (rxCrc != modbusCRC16(rx, rxLen - 2))     { g_lastFailReason = MBRTU_BAD_CRC;    return false; }

  uint8_t byteCount = rx[2];
  if (byteCount != expDataBytes)               { g_lastFailReason = MBRTU_BAD_LEN;    return false; }

  outLen = byteCount;
  memcpy(outData, &rx[3], byteCount);
  return true;
}

static bool rtuWriteSingle(uint8_t slaveId, uint8_t writeFunc, uint16_t addr, uint16_t rawVal) {
  if (g_rtuBusy) { g_lastFailReason = MBRTU_BUS_BUSY; g_lastFailLen = 0; return false; }
  RtuBusyGuard busyGuard;

  uint8_t tx[8];
  tx[0] = slaveId; tx[1] = writeFunc;
  tx[2] = (uint8_t)(addr >> 8);   tx[3] = (uint8_t)(addr & 0xFF);
  tx[4] = (uint8_t)(rawVal >> 8); tx[5] = (uint8_t)(rawVal & 0xFF);
  uint16_t crc = modbusCRC16(tx, 6);
  tx[6] = (uint8_t)(crc & 0xFF); tx[7] = (uint8_t)(crc >> 8);

  while (RS485.available()) (void)RS485.read();
  RS485.noReceive();
  RS485.beginTransmission();
  RS485.write(tx, 8);
  waitTxReallyDone();
  RS485.endTransmission();
  RS485.receive();

  uint8_t rx[8]; size_t rxLen = 0;
  unsigned long t0 = millis();
  while (millis() - t0 < S0.responseTimeoutMs) {
    while (RS485.available() && rxLen < 8) rx[rxLen++] = (uint8_t)RS485.read();
    if (rxLen >= 8) break;
    WDT_RESET_COUNTER();
    // Don't let a slow/absent RTU slave starve the Modbus TCP server for the
    // whole responseTimeoutMs window — service any pending TCP client on
    // every spin of this wait so the two paths don't fight for the CPU.
    mbTcpServerLoop();
  }
  if (rxLen < 8) return false;
  uint16_t rxCrc = (uint16_t)rx[6] | ((uint16_t)rx[7] << 8);
  return (rx[0] == slaveId) && (rx[1] == writeFunc) && (rxCrc == modbusCRC16(rx, 6));
}

// FC16 – write 2 consecutive holding registers atomically
static bool rtuWriteTwoRegs(uint8_t slaveId, uint16_t addr, uint16_t r0, uint16_t r1) {
  if (g_rtuBusy) { g_lastFailReason = MBRTU_BUS_BUSY; g_lastFailLen = 0; return false; }
  RtuBusyGuard busyGuard;

  uint8_t tx[13];
  tx[0] = slaveId; tx[1] = 0x10;
  tx[2] = (uint8_t)(addr >> 8); tx[3] = (uint8_t)(addr & 0xFF);
  tx[4] = 0x00; tx[5] = 0x02;          // qty = 2 registers
  tx[6] = 0x04;                        // byte count = 4
  tx[7] = (uint8_t)(r0 >> 8); tx[8] = (uint8_t)(r0 & 0xFF);
  tx[9] = (uint8_t)(r1 >> 8); tx[10] = (uint8_t)(r1 & 0xFF);
  uint16_t crc = modbusCRC16(tx, 11);
  tx[11] = (uint8_t)(crc & 0xFF); tx[12] = (uint8_t)(crc >> 8);

  while (RS485.available()) (void)RS485.read();
  RS485.noReceive();
  RS485.beginTransmission();
  RS485.write(tx, 13);
  waitTxReallyDone();
  RS485.endTransmission();
  RS485.receive();

  // Normal FC16 response echoes slaveId, func, addr, qty (8 bytes incl. CRC)
  uint8_t rx[8]; size_t rxLen = 0;
  unsigned long t0 = millis();
  while (millis() - t0 < S0.responseTimeoutMs) {
    while (RS485.available() && rxLen < 8) rx[rxLen++] = (uint8_t)RS485.read();
    if (rxLen >= 8) break;
    WDT_RESET_COUNTER();
    // Don't let a slow/absent RTU slave starve the Modbus TCP server for the
    // whole responseTimeoutMs window — service any pending TCP client on
    // every spin of this wait so the two paths don't fight for the CPU.
    mbTcpServerLoop();
  }
  if (rxLen < 8) return false;
  uint16_t rxCrc = (uint16_t)rx[6] | ((uint16_t)rx[7] << 8);
  return (rx[0] == slaveId) && (rx[1] == 0x10) && (rxCrc == modbusCRC16(rx, 6));
}

// ================================================================
//  32-bit combine / split according to ByteOrder
// ================================================================
static inline uint16_t byteSwap16(uint16_t v) { return (uint16_t)((v >> 8) | (v << 8)); }

static uint32_t combineRegs(uint16_t r0, uint16_t r1, uint8_t endian) {
  switch (endian) {
    case BO_LITTLE:      return ((uint32_t)r1 << 16) | r0;
    case BO_BIG_SWAP:    return ((uint32_t)byteSwap16(r0) << 16) | byteSwap16(r1);
    case BO_LITTLE_SWAP: return ((uint32_t)byteSwap16(r1) << 16) | byteSwap16(r0);
    default:             return ((uint32_t)r0 << 16) | r1;   // BO_BIG
  }
}

static void splitToRegs(uint32_t u32, uint8_t endian, uint16_t &r0, uint16_t &r1) {
  uint16_t hi = (uint16_t)(u32 >> 16);
  uint16_t lo = (uint16_t)(u32 & 0xFFFF);
  switch (endian) {
    case BO_LITTLE:      r0 = lo;             r1 = hi;             break;
    case BO_BIG_SWAP:    r0 = byteSwap16(hi); r1 = byteSwap16(lo); break;
    case BO_LITTLE_SWAP: r0 = byteSwap16(lo); r1 = byteSwap16(hi); break;
    default:             r0 = hi;             r1 = lo;             break;   // BO_BIG
  }
}

// ================================================================
//  Raw registers -> scaled value, and back
// ================================================================
static float rawToValue(const PointConfig &cfg, const uint16_t rawRegs[2]) {
  if (isCoilRtuFunc(cfg.rtuFunc)) {
    return rawRegs[0] ? 1.0f : 0.0f;
  }
  switch (cfg.format) {
    case FMT_INT16:   return (float)(int16_t)rawRegs[0] * cfg.scale;
    case FMT_INT32:   return (float)(int32_t)combineRegs(rawRegs[0], rawRegs[1], cfg.byteOrder) * cfg.scale;
    case FMT_UINT32:  return (float)combineRegs(rawRegs[0], rawRegs[1], cfg.byteOrder) * cfg.scale;
    case FMT_FLOAT32: {
      uint32_t u32 = combineRegs(rawRegs[0], rawRegs[1], cfg.byteOrder);
      float f; memcpy(&f, &u32, 4);
      return f * cfg.scale;
    }
    case FMT_BOOL:    return (rawRegs[0] & 0x0001) ? 1.0f : 0.0f;
    default:          return (float)rawRegs[0] * cfg.scale;   // FMT_UINT16
  }
}

// value -> raw registers (rawRegs[1] only used for 32-bit formats)
static void valueToRaw(const PointConfig &cfg, float value, uint16_t rawRegs[2]) {
  if (isCoilRtuFunc(cfg.rtuFunc)) {
    rawRegs[0] = (value != 0.0f) ? 1 : 0;
    return;
  }
  float unscaled = (cfg.scale != 0.0f) ? (value / cfg.scale) : value;
  switch (cfg.format) {
    case FMT_INT16:   rawRegs[0] = (uint16_t)(int16_t)lroundf(unscaled); break;
    case FMT_INT32: {
      int32_t v = (int32_t)lroundf(unscaled);
      splitToRegs((uint32_t)v, cfg.byteOrder, rawRegs[0], rawRegs[1]);
      break;
    }
    case FMT_UINT32: {
      uint32_t v = (uint32_t)lroundf(unscaled);
      splitToRegs(v, cfg.byteOrder, rawRegs[0], rawRegs[1]);
      break;
    }
    case FMT_FLOAT32: {
      float f = unscaled;
      uint32_t u32; memcpy(&u32, &f, 4);
      splitToRegs(u32, cfg.byteOrder, rawRegs[0], rawRegs[1]);
      break;
    }
    case FMT_BOOL:    rawRegs[0] = (value != 0.0f) ? 1 : 0; break;
    default:          rawRegs[0] = (uint16_t)lroundf(unscaled); break;   // FMT_UINT16
  }
}

// ================================================================
//  Public API
// ================================================================
bool mbRtuReadPoint(const PointConfig &cfg, PointRuntime &rt) {
  uint16_t qty = isCoilRtuFunc(cfg.rtuFunc) ? 1 : formatRegCount(cfg.format);

  uint8_t data[4] = {0};
  uint8_t len = 0;
  bool ok = rtuReadRaw(cfg.rtuSlaveId, cfg.rtuFunc, cfg.rtuAddr, qty, data, len);

  if (!ok) {
    if (rt.failCount < 255) rt.failCount++;
    return false;
  }

  if (isCoilRtuFunc(cfg.rtuFunc)) {
    rt.rawRegs[0] = data[0] & 0x01;
    rt.rawRegs[1] = 0;
  } else {
    rt.rawRegs[0] = ((uint16_t)data[0] << 8) | data[1];
    rt.rawRegs[1] = (qty >= 2) ? (((uint16_t)data[2] << 8) | data[3]) : 0;
  }
  rt.value = rawToValue(cfg, rt.rawRegs);
  rt.valid = true;
  rt.failCount = 0;
  rt.lastPollMs = millis();
  return true;
}

bool mbRtuWriteRaw(const PointConfig &cfg, PointRuntime &rt, const uint16_t rawRegs[2]) {
  if (!cfg.writable) return false;

  bool ok;
  if (isCoilRtuFunc(cfg.rtuFunc)) {
    bool bit = (rawRegs[0] != 0);
    ok = rtuWriteSingle(cfg.rtuSlaveId, 0x05, cfg.rtuAddr, bit ? 0xFF00 : 0x0000);
  } else {
    uint8_t qty = formatRegCount(cfg.format);
    ok = (qty == 1)
           ? rtuWriteSingle(cfg.rtuSlaveId, 0x06, cfg.rtuAddr, rawRegs[0])
           : rtuWriteTwoRegs(cfg.rtuSlaveId, cfg.rtuAddr, rawRegs[0], rawRegs[1]);
  }

  if (ok) {
    rt.rawRegs[0] = isCoilRtuFunc(cfg.rtuFunc) ? ((rawRegs[0] != 0) ? 1 : 0) : rawRegs[0];
    rt.rawRegs[1] = isCoilRtuFunc(cfg.rtuFunc) ? 0 : rawRegs[1];
    rt.value = rawToValue(cfg, rt.rawRegs);
    rt.lastPollMs = millis();
  }
  return ok;
}

bool mbRtuWriteScaled(const PointConfig &cfg, PointRuntime &rt, float engineeringValue) {
  if (!cfg.writable) return false;
  uint16_t rawRegs[2] = {0, 0};
  valueToRaw(cfg, engineeringValue, rawRegs);
  return mbRtuWriteRaw(cfg, rt, rawRegs);
}
