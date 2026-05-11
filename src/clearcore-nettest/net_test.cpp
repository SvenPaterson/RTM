// net_test.cpp — ClearCore Phase 3 Ethernet bring-up sketch.
//
// Standalone UDP echo for the ClearCore using its built-in Ethernet
// (Teknic LwIP stack via <Ethernet.h>). Mirrors the behavior of
// src/exp-board-nettest/net_test.cpp on the XPB so the same Python
// probe (tools/udp_probe.py --host 10.0.0.10) validates both peers.
//
// Built only by [env:clearcore-nettest]. Production [env:clearcore]
// does not see this file (build_src_filter excludes the folder).
//
// Behavior:
//   - Static IP 10.0.0.10, UDP port 8888.
//   - Echoes any received UDP datagram back to the sender.
//   - Prints a 1 Hz heartbeat on USB serial.
//
// Note: ClearCore firmware uses int main() (not Arduino setup/loop)
// per the existing src/clearcore/main.cpp convention.

#include <Arduino.h>
#include <Ethernet.h>

// Locally-administered MAC for ClearCore. Matches XPB pattern (...:00:10).
// The Teknic Ethernet.begin() ignores the MAC arg (uses chip MAC), but we
// keep it for documentation / cross-tool consistency.
static byte kMac[6] = { 0x02, 0x52, 0x54, 0x4D, 0x00, 0x10 };
static IPAddress kIp(10, 0, 0, 10);
static IPAddress kPeerIp(10, 0, 0, 11);  // XPB

static constexpr uint16_t kUdpPort   = 8888;
static constexpr size_t   kRxBufSize = 64;

static EthernetUDP udp;
static char rxBuf[kRxBufSize];

int main() {
    Serial.begin(115200);
    uint32_t startMs = millis();
    while (!Serial && (millis() - startMs) < 3000) { /* wait */ }

    Serial.println(F("cc-nettest boot"));

    // Bring up Ethernet (static IP). Teknic LwIP stack: blocking init.
    Ethernet.begin(kMac, kIp);

    // Wait for physical link before opening UDP socket.
    uint32_t linkStart = millis();
    while (Ethernet.linkStatus() == LinkOFF) {
        if ((millis() - linkStart) > 5000) {
            Serial.println(F("link DOWN — proceeding anyway"));
            break;
        }
    }

    uint8_t ok = udp.begin(kUdpPort);
    Serial.print(F("udp.begin -> "));
    Serial.println(ok);

    Serial.print(F("localIP="));
    Serial.println(Ethernet.localIP());
    Serial.print(F("link="));
    Serial.println(Ethernet.linkStatus() == LinkON ? 1 : 0);

    uint32_t lastMs = 0;
    uint32_t seq = 0;
    while (true) {
        // RX path — drain one packet per iteration, echo back.
        int sz = udp.parsePacket();
        if (sz > 0) {
            int n = udp.read((unsigned char*)rxBuf,
                             (sz < (int)kRxBufSize) ? sz : (int)kRxBufSize);
            Serial.print(F("RX "));
            Serial.print(n);
            Serial.print(F("B from "));
            Serial.print(udp.remoteIP());
            Serial.print(':');
            Serial.println(udp.remotePort());
            if (n > 0) {
                // Only echo to NON-peer sources (PC probe). Echoing back
                // to the peer board causes infinite ping-pong amplification.
                if (udp.remoteIP() != kPeerIp) {
                    udp.beginPacket(udp.remoteIP(), udp.remotePort());
                    udp.write((const uint8_t*)rxBuf, n);
                    udp.endPacket();
                    Serial.println(F("  echoed"));
                }
            }
        }

        // TX heartbeat.
        uint32_t now = millis();
        if (now - lastMs >= 1000) {
            lastMs = now;
            // Peer ping — CC → XPB. XPB nettest will echo it back AND mirror
            // the payload to its LCD. Cycle through some fun messages.
            static const char* kMsgs[] = {
                "HELLO XPB",
                "HOWDY",
                "PING",
                "OVER UDP",
                "@10ms",
                "RTM LIVE",
            };
            constexpr uint8_t kNumMsgs = sizeof(kMsgs) / sizeof(kMsgs[0]);
            const char* msg = kMsgs[seq % kNumMsgs];
            char ping[40];
            int n = snprintf(ping, sizeof(ping), "%s #%lu\n",
                             msg, (unsigned long)seq);
            udp.beginPacket(kPeerIp, kUdpPort);
            udp.write((const uint8_t*)ping, n);
            udp.endPacket();
            Serial.print(F("seq="));
            Serial.print(seq++);
            Serial.print(F(" link="));
            Serial.println(Ethernet.linkStatus() == LinkON ? 1 : 0);
        }

        // Keep DHCP lease alive (no-op for static IP, harmless).
        Ethernet.maintain();
    }
}
