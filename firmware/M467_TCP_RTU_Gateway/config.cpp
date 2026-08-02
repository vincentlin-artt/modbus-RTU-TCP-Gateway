// ================================================================
//  config.cpp  –  SD-card persistence + defaults + small helpers
// ================================================================
#include "config.h"
#include <Wire.h>
#include <AT24Cxx.h>
#include <math.h>

// ================================================================
//  Runtime state
// ================================================================
PointConfig  points[MAX_POINTS];
PointRuntime pointState[MAX_POINTS];

SerialConfig S0;

String sysName    = "M467-Modbus-Gateway";
String sysVersion = "1.0.0";
int    httpPort      = 80;
bool   httpEnable    = true;
int    modbusTcpPort = 502;
String loginUsername = "admin";
String loginPassword = "admin";
String loginAuthBase64;

bool   mbTcpClientMode = false;
String mbTcpClientHost = "";
int    mbTcpClientPort = 502;

bool      netDhcp = true;
IPAddress netIp(192, 168, 1, 250);
IPAddress netMask(255, 255, 255, 0);
IPAddress netGateway(192, 168, 1, 1);
IPAddress netDns(192, 168, 1, 1);

uint8_t g_macAddr[6] = {0};

String logs[MAX_LOGS];
int    logIndex = 0, logCount = 0;

volatile bool  g_rebootPending = false;
unsigned long  g_rebootAt = 0;

// ================================================================
//  Small helpers
// ================================================================
String rdLn(File &f) {
  String s = "";
  while (f.available()) {
    char c = f.read();
    if (c == '\n') break;
    if (c != '\r') s += c;
  }
  s.trim();
  return s;
}

String base64Encode(String s) {
  const char* b64 = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  String out = "";
  int val = 0, valb = -6;
  for (unsigned char c : s) {
    val = (val << 8) + c; valb += 8;
    while (valb >= 0) { out += b64[(val >> valb) & 0x3F]; valb -= 6; }
  }
  if (valb > -6) out += b64[((val << 8) >> (valb + 8)) & 0x3F];
  while (out.length() % 4) out += '=';
  return out;
}

void addLog(String msg) {
  char buf[16];
  snprintf(buf, sizeof(buf), "[%lu] ", millis());
  logs[logIndex] = String(buf) + msg;
  logIndex = (logIndex + 1) % MAX_LOGS;
  if (logCount < MAX_LOGS) logCount++;
  Serial.println(msg);
}

String ipToString(const IPAddress &ip) {
  return String(ip[0]) + "." + String(ip[1]) + "." + String(ip[2]) + "." + String(ip[3]);
}

// See config.h — mirrors Print::printFloat()'s digit-by-digit approach
// instead of String(float,decimals)'s broken dtostrf()/sprintf("%f") path.
String floatToStr(float value, uint8_t decimals) {
  if (isnan(value)) return "nan";
  if (isinf(value)) return "inf";

  bool neg = value < 0.0f;
  if (neg) value = -value;

  double rounding = 0.5;
  for (uint8_t i = 0; i < decimals; i++) rounding /= 10.0;
  double v = (double)value + rounding;

  unsigned long intPart = (unsigned long)v;
  double remainder = v - (double)intPart;

  String out = neg ? "-" : "";
  out += String(intPart);
  if (decimals > 0) {
    out += '.';
    while (decimals-- > 0) {
      remainder *= 10.0;
      unsigned int digit = (unsigned int)remainder;
      out += String(digit);
      remainder -= digit;
    }
  }
  return out;
}

uint32_t baudFromIndex(uint8_t idx) {
  static const uint32_t tbl[] = {
    300, 600, 1200, 2400, 4800, 9600,
    19200, 38400, 57600, 115200, 230400, 460800, 921600
  };
  return (idx < 13) ? tbl[idx] : 9600;
}

// ================================================================
//  MAC address — read from on-board AT24Cxx EEPROM (factory-programmed),
//  fall back to a locally-administered address if not present/valid.
// ================================================================
#define EEPROM_I2C_ADDR   0x50
#define EE_BOARD_MAC_BASE 19
static AT24Cxx gEep(EEPROM_I2C_ADDR, 32);

static bool isMacValid(const uint8_t mac[6]) {
  bool all00 = true, allFF = true;
  for (int i = 0; i < 6; i++) {
    if (mac[i] != 0x00) all00 = false;
    if (mac[i] != 0xFF) allFF = false;
  }
  if (all00 || allFF) return false;
  if (mac[0] & 0x01) return false;   // multicast bit must be 0
  return true;
}

bool initMacAddress(uint8_t macOut[6]) {
  Wire.begin();
  for (int i = 0; i < 6; i++) macOut[i] = gEep.read(EE_BOARD_MAC_BASE + i);
  if (isMacValid(macOut)) return true;

  uint8_t fallbackMac[6] = { 0x02, 0x60, 0x00, 0x00, 0xFF, 0xFF };
  memcpy(macOut, fallbackMac, 6);
  return false;
}

// ================================================================
//  initDefaults – factory defaults, used when SD is missing/blank
// ================================================================
void initDefaults() {
  S0.baudIndex   = 5;      // 9600 bps (common Modbus RTU default)
  S0.dataBits    = 8;
  S0.parity      = 0;      // None
  S0.stopBits    = 1;
  S0.preDelayUs  = 0;
  S0.postDelayUs = 0;
  // 1000ms rather than a tighter "real hardware" value: forgiving enough for
  // PC-based simulators (ModSim/ModScan etc.) that can have GUI-driven
  // latency spikes; tune down once you're talking to real RTU field devices.
  S0.responseTimeoutMs = 1000;

  sysName      = "M467-Modbus-Gateway";
  sysVersion   = "1.0.0";
  httpPort     = 80;
  httpEnable   = true;
  modbusTcpPort = 502;
  loginUsername = "admin";
  loginPassword = "admin";
  loginAuthBase64 = base64Encode(loginUsername + ":" + loginPassword);

  mbTcpClientMode = false;
  mbTcpClientHost = "";
  mbTcpClientPort = 502;

  netDhcp    = true;
  netIp      = IPAddress(192, 168, 1, 250);
  netMask    = IPAddress(255, 255, 255, 0);
  netGateway = IPAddress(192, 168, 1, 1);
  netDns     = IPAddress(192, 168, 1, 1);

  for (uint8_t i = 0; i < MAX_POINTS; i++) {
    PointConfig &p = points[i];
    p.enable   = false;
    snprintf(p.name, sizeof(p.name), "Point%02u", i + 1);
    p.rtuSlaveId     = 1;
    p.rtuFunc        = 3;              // Holding registers
    p.rtuAddr        = 0;
    p.format         = FMT_UINT16;
    p.byteOrder      = BO_BIG;
    p.scale          = 1.0f;
    p.pollIntervalMs = 1000;
    p.writable       = false;
    p.tcpUnitId      = 1;
    p.tcpRegType     = TCP_HOLDING;
    p.tcpAddr        = i;

    PointRuntime &rt = pointState[i];
    rt.rawRegs[0] = rt.rawRegs[1] = 0;
    rt.value = 0;
    rt.valid = false;
    rt.failCount = 0;
    rt.lastPollMs = 0;
    rt.nextDueMs = 0;
  }
}

// ================================================================
//  /SYSTEM.TXT
//  line1 name, line2 password, line3 httpPort, line4 modbusTcpPort,
//  line5 mbTcpClientMode(0/1), line6 mbTcpClientHost, line7 mbTcpClientPort,
//  line8 httpEnable(0/1)
// ================================================================
void loadSysConfig() {
  File f = SD.open(FILE_SYSTEM, FILE_READ);
  if (f._res != FR_OK) {
    Serial.println("[SYSTEM] not found, using defaults");
    return;
  }
  sysName  = rdLn(f);
  String pw = rdLn(f); if (pw.length() > 0) loginPassword = pw;
  String hp = rdLn(f); if (hp.length() > 0) httpPort = hp.toInt();
  String mp = rdLn(f); if (mp.length() > 0) modbusTcpPort = mp.toInt();
  String cm = rdLn(f); if (cm.length() > 0) mbTcpClientMode = (cm.toInt() == 1);
  String ch = rdLn(f); mbTcpClientHost = ch;   // may legitimately be empty
  String cp = rdLn(f); if (cp.length() > 0) mbTcpClientPort = cp.toInt();
  String he = rdLn(f); if (he.length() > 0) httpEnable = (he.toInt() == 1);
  f.close();

  if (sysName.length() == 0) sysName = "M467-Modbus-Gateway";
  if (httpPort == 0) httpPort = 80;
  if (modbusTcpPort == 0) modbusTcpPort = 502;
  if (mbTcpClientPort == 0) mbTcpClientPort = 502;
  loginAuthBase64 = base64Encode(loginUsername + ":" + loginPassword);
}

void saveSysConfig() {
  SD.remove(FILE_SYSTEM);
  File f = SD.open(FILE_SYSTEM, FILE_WRITE);
  if (f._res != FR_OK) { addLog("[SYSTEM] save failed"); return; }
  f.println(sysName);
  f.println(loginPassword);
  f.println(httpPort);
  f.println(modbusTcpPort);
  f.println(mbTcpClientMode ? 1 : 0);
  f.println(mbTcpClientHost);
  f.println(mbTcpClientPort);
  f.println(httpEnable ? 1 : 0);
  f.close();
  loginAuthBase64 = base64Encode(loginUsername + ":" + loginPassword);
}

// ================================================================
//  /NETWORK.TXT
//  line1 dhcp(0/1), line2 ip, line3 mask, line4 gateway, line5 dns
// ================================================================
void loadNetConfig() {
  File f = SD.open(FILE_NETWORK, FILE_READ);
  if (f._res != FR_OK) {
    Serial.println("[NETWORK] not found, using defaults");
    return;
  }
  netDhcp = (rdLn(f).toInt() == 1);
  String s;
  s = rdLn(f); if (s.length() > 0) netIp.fromString(s);
  s = rdLn(f); if (s.length() > 0) netMask.fromString(s);
  s = rdLn(f); if (s.length() > 0) netGateway.fromString(s);
  s = rdLn(f); if (s.length() > 0) netDns.fromString(s);
  f.close();
}

void saveNetConfig() {
  SD.remove(FILE_NETWORK);
  File f = SD.open(FILE_NETWORK, FILE_WRITE);
  if (f._res != FR_OK) { addLog("[NETWORK] save failed"); return; }
  f.println(netDhcp ? 1 : 0);
  f.println(ipToString(netIp));
  f.println(ipToString(netMask));
  f.println(ipToString(netGateway));
  f.println(ipToString(netDns));
  f.close();
}

// ================================================================
//  /SERIAL.TXT
//  line1 baudIndex, line2 dataBits, line3 parity, line4 stopBits,
//  line5 preDelayUs, line6 postDelayUs, line7 responseTimeoutMs
// ================================================================
void loadSerialConfig() {
  File f = SD.open(FILE_SERIAL, FILE_READ);
  if (f._res != FR_OK) {
    Serial.println("[SERIAL] not found, using defaults");
    return;
  }
  String s;
  s = rdLn(f); if (s.length() > 0) S0.baudIndex          = (uint8_t)s.toInt();
  s = rdLn(f); if (s.length() > 0) S0.dataBits           = (uint8_t)s.toInt();
  s = rdLn(f); if (s.length() > 0) S0.parity             = (uint8_t)s.toInt();
  s = rdLn(f); if (s.length() > 0) S0.stopBits           = (uint8_t)s.toInt();
  s = rdLn(f); if (s.length() > 0) S0.preDelayUs         = (uint16_t)s.toInt();
  s = rdLn(f); if (s.length() > 0) S0.postDelayUs        = (uint16_t)s.toInt();
  s = rdLn(f); if (s.length() > 0) S0.responseTimeoutMs  = (uint16_t)s.toInt();
  if (S0.responseTimeoutMs == 0) S0.responseTimeoutMs = 1000;
  f.close();
}

void saveSerialConfig() {
  SD.remove(FILE_SERIAL);
  File f = SD.open(FILE_SERIAL, FILE_WRITE);
  if (f._res != FR_OK) { addLog("[SERIAL] save failed"); return; }
  f.println(S0.baudIndex);
  f.println(S0.dataBits);
  f.println(S0.parity);
  f.println(S0.stopBits);
  f.println(S0.preDelayUs);
  f.println(S0.postDelayUs);
  f.println(S0.responseTimeoutMs);
  f.close();
}

// ================================================================
//  /POINTS.TXT – one line per point, comma-separated fields:
//  enable,name,rtuSlaveId,rtuFunc,rtuAddr,format,byteOrder,scale,
//  pollIntervalMs,writable,tcpUnitId,tcpRegType,tcpAddr
// ================================================================
static String csvField(const String &line, int &pos) {
  int comma = line.indexOf(',', pos);
  String field = (comma < 0) ? line.substring(pos) : line.substring(pos, comma);
  pos = (comma < 0) ? line.length() : comma + 1;
  field.trim();
  return field;
}

void loadPointsConfig() {
  File f = SD.open(FILE_POINTS, FILE_READ);
  if (f._res != FR_OK) {
    Serial.println("[POINTS] not found, using defaults");
    return;
  }
  for (uint8_t i = 0; i < MAX_POINTS; i++) {
    String line = rdLn(f);
    if (line.length() == 0) break;
    int pos = 0;
    PointConfig &p = points[i];
    p.enable = (csvField(line, pos).toInt() == 1);
    String nm = csvField(line, pos);
    strncpy(p.name, nm.c_str(), sizeof(p.name) - 1);
    p.name[sizeof(p.name) - 1] = '\0';
    p.rtuSlaveId     = (uint8_t)csvField(line, pos).toInt();
    p.rtuFunc        = (uint8_t)csvField(line, pos).toInt();
    p.rtuAddr        = (uint16_t)csvField(line, pos).toInt();
    p.format         = (uint8_t)csvField(line, pos).toInt();
    p.byteOrder      = (uint8_t)csvField(line, pos).toInt();
    p.scale          = csvField(line, pos).toFloat();
    p.pollIntervalMs = (uint16_t)csvField(line, pos).toInt();
    p.writable       = (csvField(line, pos).toInt() == 1);
    p.tcpUnitId      = (uint8_t)csvField(line, pos).toInt();
    p.tcpRegType     = (uint8_t)csvField(line, pos).toInt();
    p.tcpAddr        = (uint16_t)csvField(line, pos).toInt();
    if (p.pollIntervalMs == 0) p.pollIntervalMs = 1000;
  }
  f.close();
}

void savePointsConfig() {
  SD.remove(FILE_POINTS);
  File f = SD.open(FILE_POINTS, FILE_WRITE);
  if (f._res != FR_OK) { addLog("[POINTS] save failed"); return; }
  for (uint8_t i = 0; i < MAX_POINTS; i++) {
    PointConfig &p = points[i];
    f.print(p.enable ? 1 : 0);           f.print(',');
    f.print(p.name);                     f.print(',');
    f.print(p.rtuSlaveId);               f.print(',');
    f.print(p.rtuFunc);                  f.print(',');
    f.print(p.rtuAddr);                  f.print(',');
    f.print(p.format);                   f.print(',');
    f.print(p.byteOrder);                f.print(',');
    f.print(p.scale, 6);                 f.print(',');
    f.print(p.pollIntervalMs);           f.print(',');
    f.print(p.writable ? 1 : 0);         f.print(',');
    f.print(p.tcpUnitId);                f.print(',');
    f.print(p.tcpRegType);               f.print(',');
    f.println(p.tcpAddr);
  }
  f.close();
}

// ================================================================
//  loadAllConfig
// ================================================================
void loadAllConfig() {
  loadSysConfig();
  loadNetConfig();
  loadSerialConfig();
  loadPointsConfig();
}
