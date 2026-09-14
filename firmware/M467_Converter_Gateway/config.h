#pragma once
// ================================================================
//  config.h  –  Shared declarations for M467 TCP<->RTU Converter Gateway
//
//  Live Modbus TCP<->RTU pass-through converter on NuMaker-M467SJ_SD
//  (Arduino core "nuvoton:nuvoton"). No points table, no polling cache —
//  every TCP request becomes one live RS485 transaction in real time.
//
//  This is the Converter-only product — see the sibling
//  M467_Concentrator_Gateway sketch for the points-based polling-cache
//  product. They used to be one firmware with a runtime mode switch; split
//  into two dedicated sketches instead, since the shared/switchable code
//  path was the source of most of the hard-to-pin-down bugs.
// ================================================================

#include <Arduino.h>
#include <nvtEthernet.h>
#include <NVTSD.h>
#include <ArduinoRS485.h>
#include "NuMicro.h"
#include "board_config.h"

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

// ================================================================
//  Tunables
// ================================================================
// If httpEnable is turned off, the web UI keeps working for this long after
// boot regardless — otherwise disabling it (then rebooting, which any
// System-page save triggers) would permanently lock you out with no way
// back in short of a factory reset.
static const unsigned long HTTP_GRACE_MS     = 60000UL;
// Max simultaneous TCP connections we actively fair-schedule ourselves
// (round-robin) rather than relying on the underlying library's scan order,
// which always starts from socket 0 and can starve later connections.
static const uint8_t       MAX_TRACKED_CLIENTS = 8;

// ================================================================
//  Enums
// ================================================================
enum ConverterMode : uint8_t {
  CONV_STANDARD    = 0,  // parse real Modbus TCP (MBAP), we add/verify the RTU CRC
  CONV_TRANSPARENT = 1,  // raw byte tunnel — client already built the full RTU frame (incl. CRC)
};

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
extern bool   httpEnable;   // web UI on/off — see HTTP_GRACE_MS for the disable delay
extern int    modbusTcpPort;
extern String loginUsername, loginPassword, loginAuthBase64;

extern uint8_t converterMode;

extern bool      netDhcp;
extern IPAddress netIp, netMask, netGateway, netDns;

// MAC address (filled by initMacAddress() in setup())
extern uint8_t g_macAddr[6];

// ================================================================
//  Reboot control (used by factory reset / settings-changed reboot)
// ================================================================
extern volatile bool  g_rebootPending;
extern unsigned long  g_rebootAt;

// ================================================================
//  Watchdog pre-reset diagnostic breadcrumb (fault_handler.cpp) — a bare
//  pointer write, cheap enough to leave at the top of every major loop()
//  stage. If the device ever hangs somewhere instead of crashing, WDT_
//  IRQHandler() prints this right before the otherwise-unavoidable reset.
// ================================================================
extern volatile const char *g_wdtLastLoc;
#define WDT_MARK(loc) (g_wdtLastLoc = (loc))

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
void     saveSysConfig();
void     saveNetConfig();
void     saveSerialConfig();
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

// Diagnostics for the *last* RTU transaction that failed (read or write).
enum MbRtuFailReason : uint8_t {
  MBRTU_OK = 0,
  MBRTU_TIMEOUT,      // fewer bytes than expected arrived within the response timeout
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

// ---- modbus_rtu.cpp — generic pass-through primitives ----
// Structured passthrough: reqPdu/respPdu are Modbus PDUs (func+data, no
// slave id, no CRC) — we prepend slaveId and add/verify CRC ourselves, same
// convention as a normal RTU master. Returns false on any failure (check
// mbRtuLastFailReason()); respPduLen is set on success.
bool mbRtuRawTransaction(uint8_t slaveId, const uint8_t *reqPdu, size_t reqPduLen,
                          uint8_t *respPdu, size_t &respPduLen, size_t maxRespPduLen);
// Fully raw passthrough: rawTx is sent to RS485 byte-for-byte as given
// (caller already built slaveId+PDU+CRC themselves); rawRx is whatever came
// back, byte-for-byte, up to maxRawRxLen — no CRC check, no interpretation.
// Frame boundaries (both directions) are detected via the standard Modbus
// RTU silent-interval rule (~3.5 character times), not a length field.
bool mbRtuRawBytesTransaction(const uint8_t *rawTx, size_t rawTxLen,
                               uint8_t *rawRx, size_t &rawRxLen, size_t maxRawRxLen);
// Shared with modbus_converter.cpp's transparent sub-mode: the same
// silent-interval gap (RS485 side) is reused to delimit one client "frame"
// on the TCP side, and the same generic Stream reader works for either.
uint32_t interFrameSilenceUs();
size_t   readUntilSilence(Stream &s, uint8_t *buf, size_t maxLen,
                           unsigned long overallTimeoutMs, uint32_t gapUs);

// ---- modbus_converter.cpp ----
void  mbConverterInit();
void  mbConverterLoop();
uint16_t mbConverterActiveClientCount();

// ---- frontend.cpp ----
void  frontendInit();
void  frontendLoop();
