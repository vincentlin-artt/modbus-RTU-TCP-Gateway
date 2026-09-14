#pragma once
// ================================================================
//  modbus_tcp_server.h  –  Modbus TCP server (data-concentrator side)
//
//  Prototypes (mbTcpServerInit / mbTcpServerLoop / mbTcpActiveClientCount)
//  live in config.h. This header only carries the module-local protocol
//  constants.
// ================================================================
#include "config.h"

// Standard Modbus exception codes we actually use
#define MB_EXC_ILLEGAL_FUNCTION   0x01
#define MB_EXC_ILLEGAL_ADDRESS    0x02
#define MB_EXC_ILLEGAL_VALUE      0x03
#define MB_EXC_SERVER_FAILURE     0x04

// Bounds on a single request — generous for a 16-point gateway, keeps
// stack buffers small and bounded.
#define MB_TCP_MAX_QTY_REGS   32
#define MB_TCP_MAX_QTY_BITS   32
