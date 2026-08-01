// ================================================================
//  M467_TCP_RTU_Gateway.ino  –  Main sketch entry point
//
//  Pure Modbus RTU<->TCP data-concentrator gateway on NuMaker-M467SJ_SD.
//  16 independently-configurable points: each polls its own RS485/RTU
//  slave register/coil at its own rate; a Modbus TCP server (>=4
//  concurrent clients — see modbus_tcp_server.cpp) serves the cached
//  values and forwards writes live to the RS485 bus.
//
//  SD card config files (all-caps, matches the reference project's
//  convention):
//    /SYSTEM.TXT   – device name, login password, HTTP port, Modbus TCP port
//    /NETWORK.TXT  – DHCP / static IP settings
//    /SERIAL.TXT   – RS485 baud/bits/parity/stop + turnaround delays
//    /POINTS.TXT   – all 16 points (RTU side + TCP side mapping)
// ================================================================
#include "config.h"
#include "board.h"
#include "modbus_rtu.h"
#include "poll_engine.h"
#include "modbus_tcp_server.h"
#include "frontend.h"

uint8_t g_mac[6];

// ================================================================
//  Watchdog – init clock at boot, enable a few seconds later so SD/
//  Ethernet bring-up (which can legitimately take a moment) can't
//  trip a reset before the system is even up.
// ================================================================
static bool          g_wdtEnabled   = false;
static unsigned long g_wdtEnableAt  = 0;

static void watchdogInit() {
  SYS_UnlockReg();
  CLK_EnableModuleClock(WDT_MODULE);
  CLK_SetModuleClock(WDT_MODULE, CLK_CLKSEL1_WDTSEL_LIRC, 0);
  SYS_LockReg();
  g_wdtEnableAt = millis() + 5000;
}
static void watchdogService() {
  if (!g_wdtEnabled) {
    if ((int32_t)(millis() - g_wdtEnableAt) >= 0) {
      SYS_UnlockReg();
      WDT_Open(WDT_TIMEOUT_2POW18, 0, TRUE, FALSE);
      SYS_LockReg();
      g_wdtEnabled = true;
      addLog("[WDT] enabled");
    }
    return;
  }
  WDT_RESET_COUNTER();
}

// ================================================================
//  Factory Reset – hold PIN_RST_BTN for RST_HOLD_MS
// ================================================================
static bool     g_rstHolding    = false;
static uint32_t g_rstPressStart = 0;

static void resetToFactory() {
  Serial.println(F("\n>>> RESET TO FACTORY <<<"));
  const char *files[] = { FILE_SYSTEM, FILE_NETWORK, FILE_SERIAL, FILE_POINTS };
  for (uint8_t i = 0; i < 4; i++) {
    if (SD.exists(files[i])) SD.remove(files[i]);
  }
  Serial.println(F("[RST] Rebooting..."));
  Serial.flush();
  delay(200);
  NVIC_SystemReset();
}

static void handleRstButton() {
  bool pressed = (digitalRead(PIN_RST_BTN) == RST_ACTIVE);

  if (pressed && !g_rstHolding) {
    g_rstHolding    = true;
    g_rstPressStart = millis();
    digitalWrite(PIN_RST_LED, LOW);
  } else if (!pressed && g_rstHolding) {
    g_rstHolding = false;
    digitalWrite(PIN_RST_LED, HIGH);
  } else if (pressed && g_rstHolding) {
    if (millis() - g_rstPressStart >= RST_HOLD_MS) {
      g_rstHolding = false;
      resetToFactory();
    }
  }
}

// ================================================================
//  setup
// ================================================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println(F("\n=== M467 TCP/RTU Modbus Gateway starting==="));
  Serial.print(F("Board: ")); Serial.println(F(BOARD_FW_VERSION));   // confirms which flash actually landed

  pinMode(PIN_RST_BTN, INPUT_PULLUP);
  pinMode(PIN_RST_LED, OUTPUT);
  digitalWrite(PIN_RST_LED, HIGH);
  boardLedInit();

  initMacAddress(g_mac);
  memcpy(g_macAddr, g_mac, 6);

  Serial.print(F("SD... "));
  if (SD.begin()) {
    Serial.println(F("OK"));
    initDefaults();
    loadAllConfig();
  } else {
    Serial.println(F("FAIL - using defaults"));
    initDefaults();
    loginAuthBase64 = base64Encode(loginUsername + ":" + loginPassword);
  }

  mbRtuBegin();
  pollEngineInit();

  Serial.print(F("NET... "));
  if (netDhcp) {
    Ethernet.begin(g_mac);
    delay(1000);
  } else {
    Ethernet.begin(g_mac, netIp, netDns, netGateway, netMask);
    delay(500);
  }
  Serial.println(Ethernet.localIP());

  mbTcpServerInit();
  frontendInit();
  watchdogInit();

  addLog("System started, IP=" + ipToString(Ethernet.localIP()));
  Serial.println(F("=== Ready ===\n"));
}

// ================================================================
//  loop
// ================================================================
void loop() {
  handleRstButton();
  boardLedService();
  watchdogService();

  Ethernet.maintain();   // renew DHCP lease if needed

  frontendLoop();
  mbTcpServerLoop();
  pollEngineLoop();

  if (g_rebootPending && millis() > g_rebootAt) {
    Serial.println(F("[SYS] Rebooting..."));
    Serial.flush();
    delay(50);
    NVIC_SystemReset();
  }
}
