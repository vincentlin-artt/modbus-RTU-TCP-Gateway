// ================================================================
//  M467_Converter_Gateway.ino  –  Main sketch entry point
//
//  Live Modbus TCP<->RTU pass-through converter on NuMaker-M467SJ_SD. No
//  points table, no polling cache — every TCP request becomes one live
//  RS485 transaction in real time. Supports up to MAX_TRACKED_CLIENTS
//  simultaneous TCP connections, serviced in round-robin order (see
//  modbus_converter.cpp), in either "standard" (real Modbus TCP<->RTU
//  protocol conversion) or "transparent" (raw byte tunnel) sub-mode.
//
//  This is the Converter-only product — see the sibling
//  M467_Concentrator_Gateway sketch for the points-based polling-cache
//  product. They used to be one firmware with a runtime mode switch; split
//  into two dedicated sketches instead, since the shared/switchable code
//  path was the source of most of the hard-to-pin-down bugs.
//
//  SD card config files (all-caps, matches the reference project's
//  convention):
//    /SYSTEM.TXT   – device name, login password, HTTP port, Modbus TCP
//                    port, converter sub-mode
//    /NETWORK.TXT  – DHCP / static IP settings
//    /SERIAL.TXT   – RS485 baud/bits/parity/stop + turnaround delays
// ================================================================
#include "config.h"
#include "board.h"
#include "modbus_rtu.h"
#include "modbus_converter.h"
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
      WDT_EnableInt();
      NVIC_EnableIRQ(WDT_IRQn);
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
  const char *files[] = { FILE_SYSTEM, FILE_NETWORK, FILE_SERIAL };
  for (uint8_t i = 0; i < 3; i++) {
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
  Serial.println(F("\n=== M467 TCP<->RTU Converter Gateway starting==="));
  Serial.print(F("Board: ")); Serial.println(F(BOARD_FW_VERSION));   // confirms which flash actually landed

  // SYS->RSTSTS records *why* the last reset happened, independent of
  // whether it was our own watchdog or something else entirely (brown-out,
  // external pin, CPU lockup, ...). The WDT pre-reset IRQ (fault_handler.cpp)
  // only fires for an actual watchdog timeout, so if resets keep happening
  // without that message ever printing, this is the next thing to check —
  // read and print it here, before anything else has a chance to touch it,
  // then clear it (write-1-to-clear) so the next reset's flags are fresh.
  {
    uint32_t rstSts = SYS->RSTSTS;
    Serial.print(F("[RST] SYS->RSTSTS=0x")); Serial.print(rstSts, HEX); Serial.print(F(" ("));
    if (rstSts & SYS_RSTSTS_PORF_Msk)    Serial.print(F("POR "));
    if (rstSts & SYS_RSTSTS_PINRF_Msk)   Serial.print(F("PIN "));
    if (rstSts & SYS_RSTSTS_WDTRF_Msk)   Serial.print(F("WDT "));
    if (rstSts & SYS_RSTSTS_LVRF_Msk)    Serial.print(F("LVR "));
    if (rstSts & SYS_RSTSTS_BODRF_Msk)   Serial.print(F("BOD "));
    if (rstSts & SYS_RSTSTS_MCURF_Msk)   Serial.print(F("MCU "));
    if (rstSts & SYS_RSTSTS_HRESETRF_Msk) Serial.print(F("HRESET "));
    if (rstSts & SYS_RSTSTS_CPURF_Msk)   Serial.print(F("CPU "));
    if (rstSts & SYS_RSTSTS_CPULKRF_Msk) Serial.print(F("CPULOCKUP "));
    Serial.println(F(")"));
    SYS_UnlockReg();
    SYS->RSTSTS = rstSts;   // write-1-to-clear
    SYS_LockReg();
  }

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

  Serial.print(F("NET... "));
  if (netDhcp) {
    Ethernet.begin(g_mac);
    delay(1000);
  } else {
    Ethernet.begin(g_mac, netIp, netDns, netGateway, netMask);
    delay(500);
  }
  Serial.println(Ethernet.localIP());

  mbConverterInit();
  frontendInit();
  watchdogInit();

  addLog("System started, IP=" + ipToString(Ethernet.localIP()));
  Serial.println(F("=== Ready ===\n"));
}

// ================================================================
//  loop
// ================================================================
void loop() {
  WDT_MARK("loop:rstButton");   handleRstButton();
  WDT_MARK("loop:led");         boardLedService();
  WDT_MARK("loop:wdtService");  watchdogService();

  WDT_MARK("loop:maintain");    Ethernet.maintain();   // renew DHCP lease if needed

  WDT_MARK("loop:frontend");    frontendLoop();
  WDT_MARK("loop:converter");   mbConverterLoop();

  if (g_rebootPending && millis() > g_rebootAt) {
    Serial.println(F("[SYS] Rebooting..."));
    Serial.flush();
    delay(50);
    NVIC_SystemReset();
  }
}
