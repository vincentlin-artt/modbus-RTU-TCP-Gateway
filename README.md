# M467 TCP/RTU Modbus Gateway

Pure Modbus RTU⇄TCP data-concentrator gateway on the Nuvoton NuMaker-M467SJ_SD
(UNO R4 form-factor) board. No MQTT, DIO, TLS, or mDNS — this firmware only
speaks Modbus:

- Actively polls up to **16 independently-configurable points** over RS485
  (Modbus RTU master), each with its own slave ID, function code, register
  address, data format/byte-order/scale, and **polling rate**.
- Exposes those cached values as a **Modbus TCP server** (default port 502).
  Reads are served straight from the cache (no RS485 wait), so multiple TCP
  clients can poll concurrently without blocking each other or the bus.
  Tested to comfortably exceed 4 simultaneous TCP connections (the underlying
  lwIP stack on this board supports up to 32).
- Alternatively, **Modbus TCP Client mode** (System page): instead of
  listening, the device dials out to one remote host:port and services that
  single connection. The Modbus-protocol roles don't change — this device
  still answers requests — only which side opened the TCP socket differs.
  Useful when the SCADA/master can't connect to this device directly (behind
  a firewall/NAT). Reconnects automatically (every 10s) if the link drops.
- Writes from a TCP client (FC05/06/16) are forwarded live to the RS485 bus
  and update the cache on success.
- Built-in web UI (default port 80) for configuring everything — network,
  RS485 serial settings, and all 16 points — backed by SD-card config files.
  Can be disabled (System page) if you don't want it running in production;
  disabling never takes effect immediately (see `HTTP_GRACE_MS` in
  `config.h`) — the UI stays reachable for 60s after every boot regardless,
  so a mistaken disable can still be undone rather than permanently locking
  you out short of a factory reset.

This is a companion/sibling project to `../M467-Manager` (the Gateway
Manager / `modbus-mqtt-gateway` firmware), reusing the same board, Arduino
core, and RTU send/receive timing, but trimmed to a single purpose and
re-architected around a Modbus **TCP server** instead of an MQTT client.

## Gateway mode: Concentrator vs. Converter (System page)

Two mutually-exclusive whole-device modes — both share the same RS485 bus
and the same `modbusTcpPort`, but only one is ever actually listening:

- **Concentrator** (default, described above): 16/32-point polling cache,
  reads served instantly from cache, writes forwarded live.
- **Converter**: no points, no capacity limit — every TCP request is a live
  Modbus RTU transaction, forwarded and answered in real time. Supports up
  to `MAX_TRACKED_CLIENTS` (8) simultaneous TCP connections, each with its
  own independent timeout. Two selectable sub-modes:
  - **standard tcp↔rtu**: real Modbus TCP↔RTU protocol conversion — parses
    the MBAP header + PDU, forwards the PDU over RS485 (we add/verify the
    CRC), and translates an RTU-side failure into a Modbus exception code
    (`0x0B` gateway-target-failed-to-respond for a timeout, `0x06`
    gateway-busy if a rare same-instant bus collision occurs).
  - **transparent**: raw byte tunnel — the TCP payload the client sends IS
    the RTU frame content verbatim (client computes its own CRC). Since
    there's no length field to know where one client's frame ends, frame
    boundaries on *both* the TCP and RTU sides are detected via the
    standard Modbus RTU silent-interval rule (~3.5 character times at the
    configured RS485 baud rate) — a transparent client is expected to pace
    its bytes as if writing straight onto the RTU wire.

**Fairness**: `EthernetServer::available()` (the underlying lwIP-backed
library call) always scans its internal client list from socket index 0 and
returns the first one with pending data — under sustained traffic from a
low-indexed client that client would win every call, potentially starving
one accepted after it indefinitely. Both gateway modes work around this by
tracking distinct clients themselves (by IP+port) and dispatching service
turns in strict round-robin order (A1→B1→C1→A2→B2→...) over that list
instead of trusting scan order; `available()` is only used for discovering
new connections. In Converter mode this also means fairness for *turns at
the shared RS485 bus* specifically, since (unlike the Concentrator's
cache-based reads) every single request needs one — a client's turn is
skipped (not queued) if the bus is already busy with someone else's
transaction, and picked up again on its next turn.

## Board / point-count variants (`board_config.h`)

Two independent compile-time switches, same pattern as the sibling Gateway
Manager project's `board_config.h`:

- **`BOARD_VARIANT`**: `BOARD_METAL` or `BOARD_ABS`. The only hardware
  difference this firmware cares about is which pin drives the heartbeat
  LED — METAL uses a normal GPIO (`SYS_LED`, pin 9), ABS drives **PF6**
  instead via the VBAT-domain `RTC->GPIOCTL0` register (`board.cpp`) — the
  same register-level mechanism the reference project uses for its PF6
  backlog LED, just repurposed here since this build has no backlog concept.
- **`POINT_VARIANT`**: `POINTS_16` or `POINTS_32`. Sets `MAX_POINTS`
  everywhere (config storage, poll scheduler, TCP server lookups, web UI
  table) — no other code changes needed to go from 16 to 32 points.

Edit the two `#define`s at the top of `board_config.h` and reflash; all four
combinations (METAL/ABS × 16/32) have been compile-checked.

**Point count vs. scan time**: only one RTU transaction is ever in flight at
a time (shared RS485 bus), so a full sweep of all points takes roughly
`point_count × (single-transaction time + 20ms gap)`. Measured on this board
against a PC-based RTU simulator, a successful transaction takes ~170-210ms,
so a 16-point sweep is ~3-3.5s and a 32-point sweep ~6-7s if every read
succeeds; real RTU field devices typically respond faster than a PC
simulator, so expect better in practice. This bounds how low you can usefully
set an individual point's polling rate when many points are enabled.

## Hardware / toolchain

- Board: **NuMaker-M467SJ_SD** (`nuvoton:nuvoton:nuvoton_m467sd`), Arduino
  core `nuvoton:nuvoton` 13.3.3.
- Libraries used (all already part of the board package, except AT24Cxx):
  `nvtEthernet` (lwIP), `nvtSD` (SD card), `nvtArduinoRS485` (RS485 over
  Serial1), `Wire`, and **AT24Cxx** (on-board EEPROM MAC address — install
  from the Arduino Library Manager or copy it into your sketchbook
  `libraries/` folder if the IDE doesn't already have it).
- RS485: uses `Serial1` via the standard `ArduinoRS485` API
  (`RS485_DEFAULT_TX_PIN`/`DE`/`RE` from the board's pin definitions).
- Factory reset button: pin 3 (`INPUT_PULLUP`, hold 10s). Status LED: pin 13.
  System heartbeat LED: pin 9.

### A note on RS485 framing (baud/parity/stop bits)

This Arduino core's `RS485.begin(baud, config)` only ever programs 8-N-1 —
`UART_Open()` hardcodes the frame format and ignores the `config` argument
other than to flag RS485 auto-direction mode. To actually honor a configured
parity/stop-bits (many Modbus RTU devices default to 8E1), `mbRtuBegin()` in
`modbus_rtu.cpp` re-applies the format with the NuMicro BSP's
`UART_SetLineConfig()` directly on `UART_Desc[1].U` (the same peripheral
`Serial1`/`RS485` wraps) right after `RS485.begin()`. This is why the Serial
settings page actually has working Data Bits / Parity / Stop Bits controls,
unlike the reference project (which only ever used 8-N-1 in practice).

Relatedly: `HardwareSerial::flush()` on this core is a no-op (see
`cores/nuvoton/HardwareSerial.cpp`), so `RS485.endTransmission()` — which
calls `flush()` then immediately drops DE — can release the RS485 driver
before the last byte has actually finished shifting out, intermittently
truncating the CRC and causing the slave to silently drop the frame (bad
CRC ⇒ no response ⇒ timeout on our side, with no exception). `modbus_rtu.cpp`
works around this by polling the UART's real transmitter-empty hardware
flag (`UART_WAIT_TX_EMPTY` on `UART_Desc[1].U`) before every
`endTransmission()`, so DE never drops mid-byte.

## Building

```bash
arduino-cli compile --fqbn nuvoton:nuvoton:nuvoton_m467sd \
  --library "<path to your AT24Cxx library folder>" \
  firmware/M467_TCP_RTU_Gateway
```

Compiles cleanly (verified) — ~30% flash usage. You'll see one harmless
warning about `nvtArduinoRS485`'s declared architectures not listing
`nuvoton`; that's a stale `library.properties` tag in the vendored library,
not a real incompatibility (the reference project uses the same library the
same way).

To flash, use the Arduino IDE with the same board selected, or the
`isp_m460` upload tool referenced in `boards.txt`.

## SD card config files

Written by the web UI, read at boot. Delete any of them (or hold the reset
button 10s) to fall back to defaults.

| File           | Contents |
|----------------|----------|
| `/SYSTEM.TXT`  | device name, login password, HTTP port, Modbus TCP port, TCP client mode + remote host/port, HTTP enable, gateway mode, converter sub-mode |
| `/NETWORK.TXT` | DHCP flag, static IP, mask, gateway, DNS |
| `/SERIAL.TXT`  | RS485 baud index, data bits, parity, stop bits, pre/post delay (µs), response timeout (ms) |
| `/POINTS.TXT`  | one line per point (see `config.cpp` for the exact CSV layout) |

Default login is `admin` / `admin`, default IP is DHCP with a `192.168.1.250`
static fallback if you disable DHCP without setting anything else.

## Point model & TCP↔RTU mapping

Each of the 16 points has two independent address descriptions:

- **RTU side**: `rtuSlaveId` + `rtuFunc` (1=Coil, 2=Discrete, 3=Holding,
  4=Input) + `rtuAddr` — where the point lives on the RS485 bus. The web UI's
  RTU Addr field takes the standard 5- or 6-digit **Modicon address**
  directly (01 Coil 00001-09999, 02 Discrete 10001-19999, 03 Holding
  40001-49999, 04 Input 30001-39999 — the same labels most Modbus tools show,
  e.g. ModSim's "40001") and derives *both* `rtuFunc` and the raw 0-based
  `rtuAddr` from it (`modiconToFuncAddr()` in `frontend.cpp`, same range
  logic the reference project's `addrToFuncReg()` uses) — there's no
  separate function-code dropdown to keep in sync, since the Modicon number
  already implies it. The derived type is shown read-only next to the
  address. `PointConfig.rtuAddr`/`rtuFunc` and the RTU wire protocol code
  always work with the raw 0-based value internally.
- **TCP side**: `tcpUnitId` + `tcpRegType` + `tcpAddr` — where the point
  answers on the Modbus TCP server. `tcpAddr` is plain 0-based (not
  Modicon-converted).

**The TCP side is a raw register/coil pass-through of the RTU side** — a
Modbus TCP client reads/writes the exact same register bytes the RTU device
would send/receive. `format` / `byteOrder` / `scale` are only used for the
web dashboard's human-readable value and are not applied to the TCP wire
protocol. This matches how commercial Modbus RTU↔TCP gateways behave and
avoids ambiguity about which side is supposed to interpret scaling.

32-bit formats (INT32/UINT32/FLOAT32) occupy two consecutive addresses on
both sides (`tcpAddr` and `tcpAddr+1`). Writing a 32-bit point requires a
single FC16 (write multiple registers) request covering both addresses;
FC06 only works for 1-register points.

### Example

| Point | RTU addr (Modicon) | RTU type (derived) | Format  | TCP type | TCP addr | Writable |
|-------|---------------------|---------------------|---------|----------|----------|----------|
| Temp  | 30001               | 04 Input            | FLOAT32 | Input    | 0 (+1)   | no |
| Relay | 00001               | 01 Coil             | BOOL    | Coil     | 0        | yes |

A SCADA master reading Input Registers 0-1 at Unit ID 1 gets the two raw
registers straight from the RTU device's temperature reading; writing Coil 0
at Unit ID 1 turns the relay on/off over RS485 immediately.

## Testing

1. Flash, power up, connect to the board's IP (shown on Serial at 115200 and
   on the web dashboard) — default `admin`/`admin`.
2. `/serial` — set the RS485 baud/parity/stop bits to match your field
   devices. `/points` — configure at least one point against a real or
   simulated RTU slave (a `diagslave`/simulator on the RS485 bus, or an
   actual sensor/PLC).
3. From a PC on the same network, open **4+ simultaneous connections** with
   a Modbus TCP tool (e.g. Modbus Poll, `mbpoll -c <n>`, or several
   `mbpoll` processes at once) against the configured Unit ID/address and
   confirm all connections get timely, correct responses.
4. Try a write (FC05/06/16) against a point marked writable and confirm it
   reaches the RTU device.

## What's intentionally not here

MQTT, TLS/certificates, mDNS/UDP discovery, DIO (2DI+2DO), AWS IoT — all
present in the reference Gateway Manager project but out of scope for a
"pure Modbus" gateway. See `../M467-Manager` if you need those.
