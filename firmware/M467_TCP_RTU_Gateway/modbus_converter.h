#pragma once
// ================================================================
//  modbus_converter.h – TCP<->RTU converter mode (GW_CONVERTER)
//
//  Prototypes (mbConverterInit / mbConverterLoop / mbConverterActiveClientCount)
//  live in config.h. This header only carries module-local constants.
// ================================================================
#include "config.h"

// Modbus exception codes reported to a CONV_STANDARD client when the RTU
// side fails — the standard "gateway" codes real Modbus TCP<->RTU gateways
// use for exactly this situation (as opposed to the generic 0x04 the
// concentrator side uses for its own write failures).
#define MB_CONV_EXC_GATEWAY_BUSY         0x06   // Server Device Busy — bus already in use
#define MB_CONV_EXC_GATEWAY_NO_RESPONSE  0x0B   // Gateway Target Device Failed to Respond — RTU timeout
#define MB_CONV_EXC_SERVER_FAILURE       0x04   // bad CRC / bad header / bad length from the RTU device

// How long a CONV_TRANSPARENT client gets, once first seen ready, to finish
// delivering one RTU-frame's worth of raw bytes (paced by the RS485
// silent-interval gap — see interFrameSilenceUs()) before we give up on
// that turn. Generous relative to any real network/client-side pacing.
#define CONV_CLIENT_FRAME_TIMEOUT_MS 500UL

// How long a CONV_STANDARD client gets to finish delivering the 7-byte MBAP
// header, and separately its PDU, once the first byte of each has arrived.
#define CONV_MBAP_CHUNK_TIMEOUT_MS 200UL
