// RtmNet.h — Shared network constants for the CC↔XPB Ethernet link.
//
// Single source of truth for MACs, IPs, and the UDP port used by the
// production transport (UDP via Ethernet) and the standalone
// nettest sketches under src/clearcore-nettest/ and src/exp-board-nettest/.
//
// PC tooling (tools/udp_capture.py, tools/udp_probe.py) duplicates these
// constants in Python. Keep them in sync.
#pragma once

#include <stdint.h>

namespace RtmNet {

// Locally-administered MACs (LAA bit = 0x02 in first octet).
// Distinct per peer so a managed switch can identify them by MAC.
static constexpr uint8_t kCcMac [6] = { 0x02, 0x52, 0x54, 0x4D, 0x00, 0x10 };
static constexpr uint8_t kXpbMac[6] = { 0x02, 0x52, 0x54, 0x4D, 0x00, 0x11 };

// Static IPs on the isolated 10.0.0.0/24 rig LAN. No DHCP, no gateway.
// PC sits at 10.0.0.100 (configured on the host NIC, not in firmware).
static constexpr uint8_t kCcIp [4] = { 10, 0, 0, 10 };
static constexpr uint8_t kXpbIp[4] = { 10, 0, 0, 11 };
static constexpr uint8_t kPcIp [4] = { 10, 0, 0, 100 };

// Both peers listen on the same UDP port. Datagram src/dst IPs
// disambiguate direction; a single port keeps capture trivial.
static constexpr uint16_t kUdpPort = 8888;

// Tee outbound frames to the PC observer at kPcIp on every send.
// Cheap (one extra UDP send per frame) but makes harness/capture work
// over any unmanaged switch — switches learn MACs and only forward
// peer-to-peer frames to the addressed port, so without the tee the
// PC never sees CC↔XPB chatter. Disable by setting to 0 to drop the
// extra send on production stands where PC observation isn't wanted.
#ifndef RTM_TEE_TO_PC
#define RTM_TEE_TO_PC 1
#endif

}  // namespace RtmNet
