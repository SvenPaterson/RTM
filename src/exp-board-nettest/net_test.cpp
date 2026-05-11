// net_test.cpp
// Phase 2.0 — Flash-budget measurement sketch.
//
// Minimal Ethernet UDP echo for the Nano Every + W5500. The point of this
// sketch is NOT to be a useful runtime — it exists to force the linker to
// pull in the same UDP code paths the production firmware will eventually
// use, so we can measure the real flash cost before committing to a library.
//
// Built only by [env:exp-board-nettest]. Production [env:exp-board] does
// not see this file (build_src_filter excludes the folder).
//
// Behavior (when actually run on hardware):
//   - Static IP 192.168.1.11, port 8888.
//   - Echoes any received UDP datagram back to the sender.
//   - Prints a heartbeat counter on USB Serial once a second.
//
// Hard rules to keep the binary small:
//   - Include ONLY <SPI.h>, <Ethernet.h>, <EthernetUdp.h>.
//   - Use the static-IP overload of Ethernet.begin() — never the DHCP one
//     (the DHCP overload links the entire DHCP state machine).
//   - No EthernetClient / EthernetServer / Dns.

#include <Arduino.h>
#include <SPI.h>
#include <Ethernet3.h>
#include <EthernetUdp3.h>
#include <utility/w5500.h>
#include "LCDDriver.h"

// W5500 CS pin on the Nano Every (per migration plan).
static constexpr uint8_t kW5500CsPin  = 21; // A7
static constexpr uint8_t kW5500RstPin = 20; // A6 — drives W5500 RST low at boot

// Locally-administered MAC for the XPB.
static byte kMac[6] = { 0x02, 0x52, 0x54, 0x4D, 0x00, 0x11 };
static IPAddress kIp(10, 0, 0, 11);
static IPAddress kPeerIp(10, 0, 0, 10);  // ClearCore

static constexpr uint16_t kUdpPort = 8888;
static constexpr size_t   kRxBufSize = 64;

static EthernetUDP udp;
static char rxBuf[kRxBufSize];
static LCDDriver lcd(8);  // LCD_CS_ = 8 (matches ExpansionBoard.h)
static uint32_t rxCount = 0;

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}

    // Hardware-reset the W5500 before any SPI traffic. Guarantees clean
    // socket state across uploads (the Nano Every does NOT auto-reset the
    // chip on USB reconnect even though RST was previously tied to RESET).
    // W5500 datasheet: RST low >= 500 us, then >= 50 ms before SPI.
    pinMode(kW5500RstPin, OUTPUT);
    digitalWrite(kW5500RstPin, LOW);
    delay(10);
    digitalWrite(kW5500RstPin, HIGH);
    delay(100);

    // Park CS HIGH on every other SPI peripheral so they don't sniff
    // W5500 bus traffic (LCD goes "nuts" otherwise). Pin numbers must
    // match include/ExpansionBoard.h.
    pinMode(8,  OUTPUT); digitalWrite(8,  HIGH); // LCD_CS_
    pinMode(9,  OUTPUT); digitalWrite(9,  HIGH); // TC1_CS_
    pinMode(10, OUTPUT); digitalWrite(10, HIGH); // TC2_CS_
    pinMode(4,  OUTPUT); digitalWrite(4,  HIGH); // SD_CS_

    // Park CS HIGH before SPI starts to avoid bus contention with any
    // peripherals that share MISO. Matches spiQuiesceAll_() pattern.
    pinMode(kW5500CsPin, OUTPUT);
    digitalWrite(kW5500CsPin, HIGH);

    // --- Raw SPI sanity check BEFORE Ethernet.begin() ---
    // Read W5500 VERSIONR (common register at offset 0x0039, BSB=0).
    // Expected value per W5500 datasheet: 0x04. Anything else (0x00 / 0xFF)
    // means SPI is not talking to the chip.
    SPI.begin();
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0)); // slow 1 MHz
    digitalWrite(kW5500CsPin, LOW);
    SPI.transfer(0x00);              // address high
    SPI.transfer(0x39);              // address low (VERSIONR)
    SPI.transfer(0x00);              // control: BSB=0, RWB=0 (read), OM=00 (var len)
    uint8_t versionr = SPI.transfer(0x00);
    digitalWrite(kW5500CsPin, HIGH);
    SPI.endTransaction();
    Serial.print(F("VERSIONR=0x"));
    if (versionr < 0x10) Serial.print('0');
    Serial.println(versionr, HEX);
    Serial.println(F("(expected 0x04)"));

    Ethernet.setCsPin(kW5500CsPin);   // Ethernet3 default is pin 10; we use D21
    Ethernet.begin(kMac, kIp);   // static IP overload — DO NOT use DHCP overload
    uint8_t ok = udp.begin(kUdpPort);
    Serial.print(F("udp.begin -> "));
    Serial.println(ok);

    Serial.println(F("net_test up"));

    // --- Ethernet3 readback diagnostics ---
    IPAddress readback = Ethernet.localIP();
    Serial.print(F("localIP="));
    Serial.println(readback);
    Serial.print(F("link="));
    Serial.println(Ethernet.link());
    Serial.print(F("speed="));
    Serial.println(Ethernet.speed());

    // --- LCD bring-up (fun test: display incoming UDP messages) ---
    lcd.begin();
    lcd.setLineCenter(0, "RTM XPB nettest");
    lcd.setLineLR(1, "IP", "10.0.0.11");
    lcd.setLineLR(2, "port", "8888");
    lcd.setLineCenter(3, "waiting for CC...");
    lcd.flush();
}

void loop() {
    // RX path — drain one packet per loop, echo back.
    int sz = udp.parsePacket();
    if (sz > 0) {
        int n = udp.read(rxBuf, (sz < (int)kRxBufSize) ? sz : (int)kRxBufSize);
        Serial.print(F("RX "));
        Serial.print(n);
        Serial.print(F("B from "));
        Serial.print(udp.remoteIP());
        Serial.print(':');
        Serial.println(udp.remotePort());
        if (n > 0) {
            // Only echo to NON-peer sources (e.g., the PC probe). Echoing
            // back to the peer board causes infinite ping-pong since the
            // peer also echoes everything. Peer pings are still mirrored
            // to the LCD below.
            if (udp.remoteIP() != kPeerIp) {
                udp.beginPacket(udp.remoteIP(), udp.remotePort());
                udp.write((const uint8_t*)rxBuf, n);
                udp.endPacket();
                Serial.println(F("  echoed"));
            }

            // Show the message on the LCD. Only mirror traffic from the CC
            // peer (skip our own echoes bouncing back from CC's echo).
            if (udp.remoteIP() == kPeerIp) {
                ++rxCount;
                char line[21];
                // Null-terminate a printable copy of the payload.
                int show = (n < 20) ? n : 20;
                memcpy(line, rxBuf, show);
                line[show] = '\0';
                // Strip trailing newline for clean display.
                if (show > 0 && line[show-1] == '\n') line[show-1] = '\0';
                lcd.setLineCenter(0, "RTM XPB nettest");
                lcd.setLineLR(1, "from", "10.0.0.10");
                lcd.setLineLR(2, "msg", line);
                char cnt[12];
                snprintf(cnt, sizeof(cnt), "%lu", (unsigned long)rxCount);
                lcd.setLineLR(3, "rx", cnt);
                lcd.flush();
            }
        }
    }

    // TX heartbeat — keeps Serial linked and shows the sketch is alive.
    static uint32_t lastMs = 0;
    static uint32_t seq = 0;
    uint32_t now = millis();
    if (now - lastMs >= 1000) {
        lastMs = now;
        // Peer ping — XPB → CC. CC nettest will echo it back, then we'll
        // see an `RX ... from 10.0.0.10:8888` line in our own log.
        char ping[32];
        int n = snprintf(ping, sizeof(ping), "XPB;seq=%lu\n", (unsigned long)seq);
        udp.beginPacket(kPeerIp, kUdpPort);
        udp.write((const uint8_t*)ping, n);
        udp.endPacket();
        Serial.print(F("seq="));
        Serial.print(seq++);
        // Dump per-socket UDP state so we can see if the chip is RX'ing
        // anything even when parsePacket() returns 0.
        for (uint8_t s = 0; s < MAX_SOCK_NUM; ++s) {
            uint8_t mr = w5500.readSnMR(s);
            if (mr != SnMR::UDP) continue;
            uint16_t port = w5500.readSnPORT(s);
            uint8_t  sr   = w5500.readSnSR(s);
            uint16_t rsr  = w5500.getRXReceivedSize(s);
            Serial.print(F(" sock"));
            Serial.print(s);
            Serial.print(F(" port="));
            Serial.print(port);
            Serial.print(F(" SR=0x"));
            Serial.print(sr, HEX);
            Serial.print(F(" RSR="));
            Serial.print(rsr);
        }
        Serial.println();
    }
}
