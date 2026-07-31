#pragma once
// ================================================================
//  config.h  –  Shared declarations for M467 TCP/RTU Modbus Gateway
//
//  Pure Modbus RTU<->TCP data-concentrator gateway on NuMaker-M467SJ_SD
//  (Arduino core "nuvoton:nuvoton", same board/library stack as the
//  reference Gateway Manager project). No MQTT / DIO / TLS / mDNS here
//  on purpose — this firmware only speaks Modbus.
// ================================================================

#include <Arduino.h>
#include <nvtEthernet.h>
#include <NVTSD.h>
#include <ArduinoRS485.h>
#include "NuMicro.h"

// ================================================================
//  RST Button (Factory Reset) / System LED
// ================================================================
#define PIN_RST_BTN         3          // INPUT_PULLUP, pressed = GND
#define PIN_RST_LED         13
#define SYS_LED             9
#define SYS_LED_INTERVAL_MS 250UL      // 2 Hz blink
#define RST_HOLD_MS         10000UL    // long-press duration to factory reset
#define RST_ACTIVE          LOW

// ================================================================
//  SD card config file names
// ================================================================
#define FILE_SYSTEM   "/SYSTEM.TXT"
#define FILE_NETWORK  "/NETWORK.TXT"
#define FILE_SERIAL   "/SERIAL.TXT"
#define FILE_POINTS   "/POINTS.TXT"

// ================================================================
//  Tunables
// ================================================================
static const uint8_t       MAX_POINTS        = 16;    // fixed point count
static const uint8_t       MAX_LOGS          = 30;

// ================================================================
//  Enums
// ================================================================
enum DataFormat : uint8_t {
  FMT_INT16   = 0,
  FMT_UINT16  = 1,
  FMT_INT32   = 2,
  FMT_UINT32  = 3,
  FMT_FLOAT32 = 4,
  FMT_BOOL    = 5,
};

enum ByteOrder : uint8_t {
  BO_BIG          = 0,   // word0=high, word1=low, each word big-endian (Modbus standard)
  BO_LITTLE       = 1,   // word0=low,  word1=high
  BO_BIG_SWAP     = 2,   // word order big, bytes within each word swapped
  BO_LITTLE_SWAP  = 3,   // word order little, bytes within each word swapped
};

enum TcpRegType : uint8_t {
  TCP_COIL     = 0,   // FC01/05/15
  TCP_DISCRETE = 1,   // FC02 (read-only)
  TCP_HOLDING  = 2,   // FC03/06/16
  TCP_INPUT    = 3,   // FC04 (read-only)
};

// Number of consecutive Modbus registers a format occupies (coils/discretes are always 1 bit, N/A here)
static inline uint8_t formatRegCount(uint8_t fmt) {
  switch (fmt) {
    case FMT_INT32:
    case FMT_UINT32:
    case FMT_FLOAT32: return 2;
    default:           return 1;   // INT16 / UINT16 / BOOL
  }
}

static inline bool isCoilRtuFunc(uint8_t func) { return func == 1 || func == 2; }

// ================================================================
//  Point configuration (persisted) + runtime cache
// ================================================================
struct PointConfig {
  bool     enable;
  char     name[24];
  uint8_t  rtuSlaveId;      // RS485 target slave address 1-247
  uint8_t  rtuFunc;         // 1=Coil 2=Discrete 3=Holding 4=Input — function used against the RTU device
  uint16_t rtuAddr;         // 0-based start address on the RTU device
  uint8_t  format;          // DataFormat — interpretation for FC03/04 register points
  uint8_t  byteOrder;       // ByteOrder — only relevant for 2-register formats
  float    scale;           // read value = raw * scale; write value: raw = target / scale
  uint16_t pollIntervalMs;  // this point's own autonomous polling period
  bool     writable;        // allow TCP-side writes to be forwarded to the RTU device
  uint8_t  tcpUnitId;       // Unit ID this point answers under on the Modbus TCP server
  uint8_t  tcpRegType;      // TcpRegType — which table it is exposed on, TCP side
  uint16_t tcpAddr;         // 0-based address within that unit+table, TCP side
};

struct PointRuntime {
  uint16_t rawRegs[2];      // raw register(s) exactly as received from the RTU slave (or [0]=0/1 for coils)
  float    value;           // scale/format-interpreted value (for dashboard + write conversion)
  bool     valid;           // at least one successful poll since boot
  uint8_t  failCount;       // consecutive failed polls
  uint32_t lastPollMs;
  uint32_t nextDueMs;       // poll_engine scheduling
};

extern PointConfig  points[MAX_POINTS];
extern PointRuntime pointState[MAX_POINTS];

// ================================================================
//  Serial (RS485) port configuration
// ================================================================
struct SerialConfig {
  uint8_t  baudIndex;   // index into baudFromIndex() table
  uint8_t  dataBits;    // 5-8
  uint8_t  parity;      // 0=None 1=Odd 2=Even
  uint8_t  stopBits;    // 1 or 2
  uint16_t preDelayUs;
  uint16_t postDelayUs;
  uint16_t responseTimeoutMs;   // how long to wait for an RTU slave's reply
};

extern SerialConfig S0;

// ================================================================
//  System / network settings
// ================================================================
extern String sysName, sysVersion;
extern int    httpPort;
extern int    modbusTcpPort;
extern String loginUsername, loginPassword, loginAuthBase64;

extern bool      netDhcp;
extern IPAddress netIp, netMask, netGateway, netDns;

// MAC address (filled by initMacAddress() in setup())
extern uint8_t g_macAddr[6];

// ================================================================
//  Logger (ring buffer)
// ================================================================
extern String logs[MAX_LOGS];
extern int    logIndex, logCount;

// ================================================================
//  Reboot control (used by factory reset / settings-changed reboot)
// ================================================================
extern volatile bool  g_rebootPending;
extern unsigned long  g_rebootAt;

// ================================================================
//  Function prototypes
// ================================================================

// ---- config.cpp ----
String   rdLn(File &f);
void     initDefaults();
void     loadAllConfig();
void     loadSysConfig();
void     loadNetConfig();
void     loadSerialConfig();
void     loadPointsConfig();
void     saveSysConfig();
void     saveNetConfig();
void     saveSerialConfig();
void     savePointsConfig();
void     addLog(String msg);
String   base64Encode(String s);
bool     initMacAddress(uint8_t macOut[6]);
uint32_t baudFromIndex(uint8_t idx);
String   ipToString(const IPAddress &ip);   // this core's IPAddress has no toString()
// String(float, decimals) goes through dtostrf()->sprintf("%f"), which this
// core links with -specs=nano.specs (no _printf_float patch) and silently
// produces an EMPTY string for any float — confirmed empirically. Print::
// print(double, digits) works fine (it has its own digit-by-digit
// printFloat(), no sprintf involved) but String has no equivalent, so use
// this wherever a float needs to end up in a String (log lines etc).
String   floatToStr(float value, uint8_t decimals);

// ---- modbus_rtu.cpp ----
void  mbRtuBegin();
bool  mbRtuReadPoint(const PointConfig &cfg, PointRuntime &rt);
// Raw pass-through write (what a Modbus TCP client sends is exactly what goes
// out on the RTU wire — same convention as reads). rawRegs[1] only used for
// 2-register formats.
bool  mbRtuWriteRaw(const PointConfig &cfg, PointRuntime &rt, const uint16_t rawRegs[2]);
// Convenience for the web UI: takes an engineering value, applies scale/format
// to derive the raw register(s), then calls mbRtuWriteRaw().
bool  mbRtuWriteScaled(const PointConfig &cfg, PointRuntime &rt, float engineeringValue);

// Diagnostics for the *last* RTU transaction that failed (read or write) —
// lets poll_engine log something more useful than just "read failed".
enum MbRtuFailReason : uint8_t {
  MBRTU_OK = 0,
  MBRTU_TIMEOUT,      // fewer bytes than expected arrived within MB_RTU_TIMEOUT_MS
  MBRTU_EXCEPTION,    // slave replied with a valid Modbus exception frame
  MBRTU_BAD_HEADER,   // response slave id / func doesn't match the request
  MBRTU_BAD_CRC,
  MBRTU_BAD_LEN,      // byte count field didn't match what we expected
  MBRTU_BUS_BUSY,     // rejected instantly — another RTU transaction was already in flight
};
uint8_t mbRtuLastFailReason();
uint8_t mbRtuLastFailLen();
uint8_t mbRtuLastFailExceptionCode();
String  mbRtuFailReasonText(uint8_t reason);
bool    mbRtuIsBusy();   // true while an RTU transaction (read or write) is in flight

// ---- poll_engine.cpp ----
void  pollEngineInit();
void  pollEngineLoop();

// ---- modbus_tcp_server.cpp ----
void  mbTcpServerInit();
void  mbTcpServerLoop();
uint16_t mbTcpActiveClientCount();

// ---- frontend.cpp ----
void  frontendInit();
void  frontendLoop();
