#pragma once
// ================================================================
//  board_config.h — hardware/point-capacity variant switches
//
//  Same pattern as the sibling Gateway Manager project's board_config.h,
//  trimmed to what actually differs in this pure-Modbus build (no
//  display/DIO/backlog here). Two independent axes:
//
//    BOARD_VARIANT : METAL or ABS enclosure. The only real hardware
//                    difference for this firmware is which pin drives
//                    the system heartbeat LED — METAL uses a normal
//                    GPIO (SYS_LED, see config.h), ABS drives PF6
//                    instead via the VBAT-domain GPIOCTL0 register
//                    (same register-level mechanism the reference
//                    project uses for its PF6 backlog LED).
//    POINT_VARIANT : 16 or 32 Modbus points.
//
//  ★★★ Pick the build here ★★★
// ================================================================

#define BOARD_METAL   1     // Backlog/heartbeat LED on normal GPIO
#define BOARD_ABS     2     // Heartbeat LED on PF6 (VBAT domain)

#ifndef BOARD_VARIANT
  #define BOARD_VARIANT   2     // <- switch BOARD_METAL / BOARD_ABS here
#endif

#define POINTS_16  16
#define POINTS_32  32

#ifndef POINT_VARIANT
  #define POINT_VARIANT   POINTS_16       // <- switch POINTS_16 / POINTS_32 here
#endif

#if POINT_VARIANT == POINTS_16
  #define POINT_COUNT         16
  #define POINT_VARIANT_NAME  "s16p"
#elif POINT_VARIANT == POINTS_32
  #define POINT_COUNT         32
  #define POINT_VARIANT_NAME  "s32p"
#else
  #error "board_config.h: POINT_VARIANT must be POINTS_16 or POINTS_32"
#endif

// ================================================================
//  Expands automatically from BOARD_VARIANT — no need to touch below
//
//  HAS_DISPLAY / HAS_DIO_MODULE are placeholders for parity with the
//  reference project's board_config.h — this pure-Modbus build has no
//  display/LCD/OLED or 2DI+2DO module code at all yet, so both are 0 on
//  every variant right now. Wire them to real modules (and set METAL=1)
//  if/when that code gets added here.
// ================================================================
#if BOARD_VARIANT == BOARD_METAL
  #define HAS_DISPLAY         0     // no display module in this build (placeholder)
  #define HAS_DIO_MODULE      0     // no 2DI+2DO module in this build (placeholder)
  #define HAS_BACKLOG_GPIO    1     // heartbeat LED on its own GPIO
  #define LED_BACKLOG_PIN     SYS_LED
  #define BOARD_VARIANT_NAME  POINT_VARIANT_NAME "-MTL"
  #define BOARD_FW_VERSION    "v1.0.0-MTL"

#elif BOARD_VARIANT == BOARD_ABS
  #define HAS_DISPLAY         0     // no display module in this build (placeholder)
  #define HAS_DIO_MODULE      0     // no 2DI+2DO module in this build (placeholder)
  #define HAS_BACKLOG_GPIO    0     // heartbeat LED via PF6 instead (boardLedSet(), board.cpp)
  #define BOARD_VARIANT_NAME  POINT_VARIANT_NAME "-ABS"
  #define BOARD_FW_VERSION    "v1.0.7-ABS"

#else
  #error "board_config.h: BOARD_VARIANT must be BOARD_METAL or BOARD_ABS"
#endif
