# Ethernet3 — RTM local patch

This is a vendored copy of [sstaub/Ethernet3](https://github.com/sstaub/Ethernet3) v1.6.0
with one local change made for the RTM project's flash budget.

## Patch: stub `EthernetUDP::beginPacket(const char*, uint16_t)`

File: [src/EthernetUdp3.cpp](src/EthernetUdp3.cpp) — the DNS-resolving overload
of `beginPacket()` is replaced with a no-op stub that returns `0`.

### Why
The Arduino Nano Every (`XPB`) has 47.5 KB of flash. With
`RTM_LINK_ETHERNET=1`, the upstream library lands at 50,797 B (104.4 % of
budget). The `(const char*, port)` overload pulls in `DNSClient` and
`getHostByName()` for ~1,720 B of code that this project never executes — we
always call `beginPacket(IPAddress, port)` directly with the static peer
address from [include/RtmNet.h](../../include/RtmNet.h). Stubbing the unused
overload drops the build to 47,855 B (98.4 %), inside budget.

### Maintenance
If this library is ever re-vendored from upstream, re-apply the patch shown in
[src/EthernetUdp3.cpp](src/EthernetUdp3.cpp) (it is annotated with
`RTM patch:` for grep-ability) or build will exceed flash.

## Patch: cap W5500 SPI clock at 4 MHz (RTM override)

File: [src/utility/w5500.cpp](src/utility/w5500.cpp) — `wiznet_SPI_settings`
clock changed from upstream 8 MHz to `RTM_W5500_SPI_HZ` (default 4 MHz).

### Why
Wireshark + observer captures on 2026-05-19 proved the XPB W5500 was
producing corrupted UDP TX (`Len=120/206/256` payloads filled with
`\x90\xXX` AVR flash-pattern bytes) and silently dropping most inbound
unicast `REQ:PROTO` frames at the upstream 8 MHz SPI clock with the
current Nano Every + jumper-wire bring-up. Capping the clock at 4 MHz
provides margin for the present wiring; the value is build-time
overridable via `-DRTM_W5500_SPI_HZ=<hz>` so it can be raised once the
production XPB layout / decoupling is verified clean.

> Update 2026-05-19 (later): all XPB build envs (`XPB`, `XPB_PROD_TRACE`,
> `XPB_DEBUG`) now pin `RTM_W5500_SPI_HZ=2000000UL` (2 MHz) because 4 MHz
> still produced intermittent SPI wedges on the current bring-up. 1 MHz was
> tested but made boot-completion -> first-tick worse, so 2 MHz is the
> current sweet spot. See the busy-wait bound patches below; those are what
> actually let the MCU survive a wedge instead of locking.

## Patch: bound `sendUDP()` SEND_OK busy-wait (RTM)

File: [src/utility/socket.cpp](src/utility/socket.cpp) — `sendUDP()` SnIR
`SEND_OK`/`TIMEOUT` polling loop now exits after `RTM_W5500_SEND_TIMEOUT_MS`
(default 60 ms).

### Why
The stock loop spins `while ((readSnIR(s) & SEND_OK) != SEND_OK)` with no
upper bound. On the current XPB bring-up the W5500 occasionally fails to set
`SEND_OK` (the SnIR read returns garbage or the IC is half-wedged after a
shared-bus SPI glitch). That spin froze the MCU entirely, which in turn
froze MAX31855 reads and starved the LCD/state machine. The patch returns 0
to the caller on timeout after clearing SnIR; the caller's own retry
machinery (`RtmComms` reliability layer + PROTO periodic retransmit) is
responsible for redrive.

## Patch: bound `execCmdSn()` command-register polling (RTM)

File: [src/utility/w5500.cpp](src/utility/w5500.cpp) — `execCmdSn()` now
exits its `while (readSnCR(s))` wait after `RTM_W5500_CMD_TIMEOUT_MS`
(default 20 ms).

### Why
Every `Sock_OPEN` / `Sock_RECV` / `Sock_SEND` / `Sock_CLOSE` writes the
command register and then waits for the chip to clear it. If SPI returns
nonzero garbage on the readback even once, the loop spins forever. That
was the dominant `tick()` hang point after the `sendUDP` patch above (the
RX path issues `Sock_RECV` every iteration via `recv()` →
`parsePacket()`).

## Patch: bound `getRXReceivedSize()` / `getTXFreeSize()` double-read (RTM)

File: [src/utility/w5500.cpp](src/utility/w5500.cpp) — the W5500-recommended
"read twice until two consecutive reads match" loops are now capped at
`RTM_W5500_FSR_MAX_ATTEMPTS` (default 16) iterations.

### Why
On a healthy bus those loops converge in 1-2 reads. On a noisy or wedged
SPI bus they may never converge, producing the same hard MCU lockup. After
16 attempts we accept the most recent read; a brief over/under-report is
recoverable, a hang is not.

### Combined effect
With all three busy-wait bounds in place plus the 2 MHz SPI cap, XPB now
boots, completes `begin()`, and runs sustained `tick()` heartbeats with
working RUN/RESET switch publishing and CC-side `[LINK] UP` / E-STOP
recovery (verified by `tools/rig_trace.py` capture
`test/log/2026/05/19/171656_rig_trace.log`). The system is still
intermittently susceptible to a hard wedge that the bounded loops can't
revive without re-initializing the W5500; full SPI reliability is expected
to require board-level decoupling/layout work on the production XPB PCB.
