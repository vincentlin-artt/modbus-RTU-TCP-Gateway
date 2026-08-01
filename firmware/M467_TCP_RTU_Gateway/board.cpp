// ================================================================
//  board.cpp — heartbeat LED, routed per BOARD_VARIANT (board_config.h)
//
//  HAS_BACKLOG_GPIO=1 (BOARD_METAL): normal GPIO (LED_BACKLOG_PIN) via
//    digitalWrite().
//  HAS_BACKLOG_GPIO=0 (BOARD_ABS)  : PF6, a VBAT-domain pin only reachable
//    through the RTC->GPIOCTL0 register (not a normal Arduino pin) — same
//    register-level mechanism as the reference Gateway Manager project's
//    PF6 backlog LED. Active low.
// ================================================================
#include "board.h"

#if !HAS_BACKLOG_GPIO

static void pf6Init() {
  // IOCTLSEL = 1 lets the VBAT domain (PF4-PF11) drive GPIOCTL0/1 instead
  // of the normal GPIO controller.
  RTC->LXTCTL |= RTC_LXTCTL_IOCTLSEL_Msk;
  // PF6 -> GPIOCTL0 bit-pair 2 (OPMODE2). 01 = output push-pull.
  RTC->GPIOCTL0 = (RTC->GPIOCTL0 & ~RTC_GPIOCTL0_OPMODE2_Msk)
                | (0x1UL << RTC_GPIOCTL0_OPMODE2_Pos);
  RTC->GPIOCTL0 |= RTC_GPIOCTL0_DOUT2_Msk;   // start off (active low: HIGH = off)
}

static void pf6Set(bool on) {
  if (on) RTC->GPIOCTL0 &= ~RTC_GPIOCTL0_DOUT2_Msk;   // LOW  = on
  else    RTC->GPIOCTL0 |=  RTC_GPIOCTL0_DOUT2_Msk;    // HIGH = off
}

#endif

void boardLedInit() {
#if !HAS_BACKLOG_GPIO
  pf6Init();
#else
  pinMode(LED_BACKLOG_PIN, OUTPUT);
  digitalWrite(LED_BACKLOG_PIN, LOW);
#endif
}

void boardLedSet(bool on) {
#if !HAS_BACKLOG_GPIO
  pf6Set(on);
#else
  digitalWrite(LED_BACKLOG_PIN, on ? HIGH : LOW);
#endif
}

void boardLedService() {
  static unsigned long lastMs = 0;
  static bool          ledOn  = false;
  unsigned long now = millis();
  if (now - lastMs >= SYS_LED_INTERVAL_MS) {
    lastMs = now;
    ledOn = !ledOn;
    boardLedSet(ledOn);
  }
}
