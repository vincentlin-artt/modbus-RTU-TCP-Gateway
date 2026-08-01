#pragma once
// ================================================================
//  board.h — abstracts the one real hardware difference between the
//  BOARD_METAL / BOARD_ABS variants (see board_config.h): which pin
//  drives the system heartbeat LED.
// ================================================================
#include "config.h"

void boardLedInit();          // call once from setup()
void boardLedSet(bool on);    // on/off — abstracts GPIO (METAL) vs PF6 (ABS)
// Non-blocking 2Hz heartbeat toggle. Call from the main loop() *and* from
// inside any long blocking wait (e.g. modbus_rtu.cpp's RTU response waits,
// which can run up to the configured response timeout) — otherwise the
// blink stalls and jumps whenever loop() is stuck waiting on RS485,
// which is what made it look unstable.
void boardLedService();
