// ================================================================
//  fault_handler.cpp – diagnostic Cortex-M4 fault handlers
//
//  This core's default HardFault_Handler (weak-aliased to Dummy_Handler in
//  startup_M460.c) is just `while(1){}` — no watchdog feed inside it, so a
//  genuine crash (bad pointer deref, stack overflow, etc.) looks IDENTICAL
//  in the Serial log to an application hang: the CPU just spins there
//  until the already-running watchdog eventually resets it, ~26s later,
//  with zero indication anything actually went wrong versus just being
//  slow. Overriding these (they're weak symbols, so a strong definition
//  here in the sketch takes priority) prints the faulting PC/LR and the
//  Cortex-M fault status registers before forcing an immediate reset, so
//  the next crash tells us exactly where it happened instead of just
//  "the device rebooted."
//
//  Cross-reference the printed PC against the compiled .elf, e.g.:
//    arm-none-eabi-addr2line -e <sketch>.ino.elf -f -C 0x<PC>
//  (the .elf lives in the arduino-cli build output / Arduino IDE's
//  temporary build directory for this sketch).
// ================================================================
#include "NuMicro.h"
#include <Arduino.h>

extern "C" void printFaultInfo(uint32_t *stackedRegs) {
  Serial.println();
  Serial.println(F("!!! FAULT !!!"));
  Serial.print(F("R0   = 0x")); Serial.println(stackedRegs[0], HEX);
  Serial.print(F("R1   = 0x")); Serial.println(stackedRegs[1], HEX);
  Serial.print(F("R2   = 0x")); Serial.println(stackedRegs[2], HEX);
  Serial.print(F("R3   = 0x")); Serial.println(stackedRegs[3], HEX);
  Serial.print(F("R12  = 0x")); Serial.println(stackedRegs[4], HEX);
  Serial.print(F("LR   = 0x")); Serial.println(stackedRegs[5], HEX);
  Serial.print(F("PC   = 0x")); Serial.println(stackedRegs[6], HEX);
  Serial.print(F("PSR  = 0x")); Serial.println(stackedRegs[7], HEX);
  Serial.print(F("CFSR = 0x")); Serial.println(SCB->CFSR, HEX);
  Serial.print(F("HFSR = 0x")); Serial.println(SCB->HFSR, HEX);
  Serial.print(F("MMFAR= 0x")); Serial.println(SCB->MMFAR, HEX);
  Serial.print(F("BFAR = 0x")); Serial.println(SCB->BFAR, HEX);
  Serial.println(F("PC is the important one -- cross-reference against the .elf with addr2line."));

  // delay()/millis() rely on SysTick, which can't fire while we're in a
  // fault handler (highest exception priority) -- a plain busy-wait is the
  // only thing that reliably gives the (polling) UART time to push these
  // bytes out before we force the reset ourselves.
  for (volatile uint32_t i = 0; i < 3000000UL; i++) { }

  NVIC_SystemReset();
}

// Standard Cortex-M pattern: figure out whether the exception used MSP or
// PSP (bit 2 of the EXC_RETURN value in LR) and pass that stack pointer
// (where the CPU auto-stacked r0-r3/r12/lr/pc/psr on exception entry) to
// printFaultInfo() as its argument (r0), per AAPCS. `naked` because we
// must not let the compiler generate a normal prologue here.
extern "C" __attribute__((naked)) void HardFault_Handler(void) {
  __asm volatile (
    "tst lr, #4 \n"
    "ite eq \n"
    "mrseq r0, msp \n"
    "mrsne r0, psp \n"
    "b printFaultInfo \n"
  );
}

extern "C" __attribute__((naked)) void BusFault_Handler(void) {
  __asm volatile (
    "tst lr, #4 \n"
    "ite eq \n"
    "mrseq r0, msp \n"
    "mrsne r0, psp \n"
    "b printFaultInfo \n"
  );
}

extern "C" __attribute__((naked)) void UsageFault_Handler(void) {
  __asm volatile (
    "tst lr, #4 \n"
    "ite eq \n"
    "mrseq r0, msp \n"
    "mrsne r0, psp \n"
    "b printFaultInfo \n"
  );
}

extern "C" __attribute__((naked)) void MemManage_Handler(void) {
  __asm volatile (
    "tst lr, #4 \n"
    "ite eq \n"
    "mrseq r0, msp \n"
    "mrsne r0, psp \n"
    "b printFaultInfo \n"
  );
}

// ================================================================
//  Watchdog pre-reset diagnostic
//
//  If the device is genuinely stuck in a loop somewhere (not a fault --
//  see above), the CPU never returns to code that could print anything on
//  its own; the fault handlers above are silent for exactly this reason.
//  This WDT hardware can fire an interrupt one reset-delay period (~1026
//  WDT clocks, ~100ms at the LIRC rate this runs at) before the reset
//  it's already committed to, which is enough time for one short print --
//  see WDT_MARK()/g_wdtLastLoc in config.h, updated (a bare pointer write,
//  cheap enough to sprinkle liberally) at the top of every major loop()
//  stage. This handler doesn't touch WDT->RSTCNT: the reset that's already
//  queued should still happen, this only reports where things were right
//  before it does.
// ================================================================
extern "C" volatile const char *g_wdtLastLoc = "boot";

extern "C" void WDT_IRQHandler(void) {
  if (WDT_GET_TIMEOUT_INT_FLAG()) {
    WDT_CLEAR_TIMEOUT_INT_FLAG();
    Serial.print(F("\n!!! WDT about to reset -- last loc: "));
    Serial.println((const char *)g_wdtLastLoc);
  }
}
