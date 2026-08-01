#pragma once
// ================================================================
//  board.h — abstracts the one real hardware difference between the
//  BOARD_METAL / BOARD_ABS variants (see board_config.h): which pin
//  drives the system heartbeat LED.
// ================================================================
#include "config.h"

void boardLedInit();          // call once from setup()
void boardLedSet(bool on);    // on/off — abstracts GPIO (METAL) vs PF6 (ABS)
