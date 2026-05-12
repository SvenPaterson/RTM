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
