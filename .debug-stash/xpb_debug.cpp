#include <Arduino.h>
#include <SPI.h>
#include <Ethernet3.h>
#include <EthernetUdp3.h>

#include "RtmNet.h"

namespace {
constexpr uint8_t kLcdCsPin = 8;
constexpr uint8_t kSdCsPin = 4;
constexpr uint8_t kTc1CsPin = 9;
constexpr uint8_t kTc2CsPin = 10;
constexpr uint8_t kSpiMasterGuardPin = 10;
constexpr uint8_t kW5500CsPin = 21;   // A7
constexpr uint8_t kW5500RstPin = 20;  // A6

constexpr uint32_t kLinkPollMs = 500;
constexpr uint32_t kHeartbeatMs = 1000;
constexpr uint8_t kMaxPacketsPerLoop = 6;
constexpr size_t kRxBufSize = 256;

EthernetUDP gUdp;
bool gUdpReady = false;
bool gLinkUp = false;

uint32_t gLastLinkPollMs = 0;
uint32_t gLastHeartbeatMs = 0;

uint32_t gSeq = 0;
uint32_t gRxCount = 0;
uint32_t gTxCount = 0;
uint32_t gEchoCount = 0;
uint32_t gDropCount = 0;
uint32_t gCcRxCount = 0;
uint32_t gCcHbCount = 0;
uint32_t gCcReqProtoCount = 0;

IPAddress gPcIp(RtmNet::kPcIp[0], RtmNet::kPcIp[1], RtmNet::kPcIp[2], RtmNet::kPcIp[3]);
IPAddress gObsIp(RtmNet::kObserverBroadcastIp[0],
                 RtmNet::kObserverBroadcastIp[1],
                 RtmNet::kObserverBroadcastIp[2],
                 RtmNet::kObserverBroadcastIp[3]);

bool isCcIp(const IPAddress &ip) {
    return ip[0] == RtmNet::kCcIp[0] &&
           ip[1] == RtmNet::kCcIp[1] &&
           ip[2] == RtmNet::kCcIp[2] &&
           ip[3] == RtmNet::kCcIp[3];
}

bool isPcIp(const IPAddress &ip) {
    return ip[0] == RtmNet::kPcIp[0] &&
           ip[1] == RtmNet::kPcIp[1] &&
           ip[2] == RtmNet::kPcIp[2] &&
           ip[3] == RtmNet::kPcIp[3];
}

bool startsWith(const char *buf, size_t len, const char *prefix) {
    size_t i = 0;
    while (prefix[i] != '\0') {
        if (i >= len || buf[i] != prefix[i]) {
            return false;
        }
        ++i;
    }
    return true;
}

void parkSpiChipSelects() {
    pinMode(kSdCsPin, OUTPUT);
    digitalWrite(kSdCsPin, HIGH);

    pinMode(kLcdCsPin, OUTPUT);
    digitalWrite(kLcdCsPin, HIGH);

    pinMode(kTc1CsPin, OUTPUT);
    digitalWrite(kTc1CsPin, HIGH);

    pinMode(kTc2CsPin, OUTPUT);
    digitalWrite(kTc2CsPin, HIGH);

    pinMode(kW5500CsPin, OUTPUT);
    digitalWrite(kW5500CsPin, HIGH);

    pinMode(kSpiMasterGuardPin, OUTPUT);
    digitalWrite(kSpiMasterGuardPin, HIGH);

    SPI.begin();
}

void resetW5500() {
    pinMode(kW5500RstPin, OUTPUT);
    digitalWrite(kW5500RstPin, LOW);
    delay(10);
    digitalWrite(kW5500RstPin, HIGH);
    delay(100);
}

void initEthernet() {
    Ethernet.setCsPin(kW5500CsPin);
    uint8_t mac[6];
    for (uint8_t i = 0; i < 6; ++i) {
        mac[i] = RtmNet::kXpbMac[i];
    }
    IPAddress ip(RtmNet::kXpbIp[0], RtmNet::kXpbIp[1], RtmNet::kXpbIp[2], RtmNet::kXpbIp[3]);
    Ethernet.begin(mac, ip);
}

bool sendLine(const IPAddress &ip, uint16_t port, const char *line) {
    if (!gUdpReady || !gLinkUp) {
        return false;
    }
    if (!gUdp.beginPacket(ip, port)) {
        return false;
    }
    gUdp.write(reinterpret_cast<const uint8_t *>(line), strlen(line));
    return gUdp.endPacket() == 1;
}

void sendHeartbeat() {
    char line[224];
    snprintf(line, sizeof(line),
             "XPBDBG;SEQ=%lu;LINK=%u;RX=%lu;TX=%lu;ECHO=%lu;DROP=%lu;CCRX=%lu;CCHB=%lu;CCREQ=%lu\n",
             static_cast<unsigned long>(gSeq++),
             static_cast<unsigned>(gLinkUp ? 1U : 0U),
             static_cast<unsigned long>(gRxCount),
             static_cast<unsigned long>(gTxCount),
             static_cast<unsigned long>(gEchoCount),
             static_cast<unsigned long>(gDropCount),
             static_cast<unsigned long>(gCcRxCount),
             static_cast<unsigned long>(gCcHbCount),
             static_cast<unsigned long>(gCcReqProtoCount));

    // Keep serial telemetry flowing even when link is down.
    Serial.print(line);

    if (sendLine(gObsIp, RtmNet::kObserverPort, line)) {
        ++gTxCount;
    }
    if (sendLine(gPcIp, RtmNet::kObserverPort, line)) {
        ++gTxCount;
    }
}

void sendLinkEvent(bool up) {
    char line[96];
    snprintf(line, sizeof(line), "XPBDBG;LINK=%s\n", up ? "UP" : "DOWN");

    // Emit to USB first so link-down transitions are always visible.
    Serial.print(line);

    if (sendLine(gObsIp, RtmNet::kObserverPort, line)) {
        ++gTxCount;
    }
    if (sendLine(gPcIp, RtmNet::kObserverPort, line)) {
        ++gTxCount;
    }
}

void sendEcho(const IPAddress &remoteIp, uint16_t remotePort, size_t len) {
    char line[96];
    snprintf(line, sizeof(line), "XPBDBG;ECHO;LEN=%u;RX=%lu\n",
             static_cast<unsigned>(len),
             static_cast<unsigned long>(gRxCount));
    if (sendLine(remoteIp, remotePort, line)) {
        ++gTxCount;
        ++gEchoCount;
    }
}

bool isObserverBeacon(const char *buf, size_t len) {
    if (len < RtmNet::kObserverBeaconLen) {
        return false;
    }
    for (uint8_t i = 0; i < RtmNet::kObserverBeaconLen; ++i) {
        if (buf[i] != RtmNet::kObserverBeacon[i]) {
            return false;
        }
    }
    return true;
}

void pumpRx() {
    if (!gUdpReady) {
        return;
    }

    int sz = gUdp.parsePacket();
    uint8_t packets = 0;
    while (sz > 0 && packets < kMaxPacketsPerLoop) {
        ++packets;
        ++gRxCount;

        IPAddress remoteIp = gUdp.remoteIP();
        uint16_t remotePort = gUdp.remotePort();
        const bool fromCc = isCcIp(remoteIp);
        const bool fromPc = isPcIp(remoteIp);

        char buf[kRxBufSize];
        int n = gUdp.read(reinterpret_cast<uint8_t *>(buf), sizeof(buf) - 1);
        if (n < 0) {
            n = 0;
        }
        buf[n] = '\0';

        while (gUdp.available() > 0) {
            gUdp.read();
        }

        if (!(fromCc || fromPc)) {
            ++gDropCount;
            Serial.print("XPBDBG;RX_DROP;IP=");
            Serial.print(remoteIp[0]);
            Serial.print('.');
            Serial.print(remoteIp[1]);
            Serial.print('.');
            Serial.print(remoteIp[2]);
            Serial.print('.');
            Serial.print(remoteIp[3]);
            Serial.print(";PORT=");
            Serial.print(remotePort);
            Serial.print(";LEN=");
            Serial.print(n);
            Serial.print(";COUNT=");
            Serial.println(static_cast<unsigned long>(gDropCount));
        } else if (fromPc && isObserverBeacon(buf, static_cast<size_t>(n))) {
            if (sendLine(remoteIp, remotePort, "XPBDBG;OBS=ACK\n")) {
                ++gTxCount;
            }
        } else {
            if (fromCc) {
                ++gCcRxCount;

                if (startsWith(buf, static_cast<size_t>(n), "HB;")) {
                    ++gCcHbCount;
                    Serial.print("XPBDBG;CC_HB_RX;COUNT=");
                    Serial.println(static_cast<unsigned long>(gCcHbCount));
                }
                if (startsWith(buf, static_cast<size_t>(n), "REQ:PROTO")) {
                    ++gCcReqProtoCount;
                    Serial.print("XPBDBG;CC_REQ_PROTO_RX;COUNT=");
                    Serial.println(static_cast<unsigned long>(gCcReqProtoCount));
                }

                sendEcho(remoteIp, remotePort, static_cast<size_t>(n));
            }
        }

        Serial.print("XPBDBG;RX;IP=");
        Serial.print(remoteIp[0]);
        Serial.print('.');
        Serial.print(remoteIp[1]);
        Serial.print('.');
        Serial.print(remoteIp[2]);
        Serial.print('.');
        Serial.print(remoteIp[3]);
        Serial.print(";PORT=");
        Serial.print(remotePort);
        Serial.print(";LEN=");
        Serial.println(n);

        sz = gUdp.parsePacket();
    }
}

void pollLink() {
    const uint32_t now = millis();
    if ((now - gLastLinkPollMs) < kLinkPollMs) {
        return;
    }
    gLastLinkPollMs = now;

    const bool linkUpNow = (Ethernet.link() == 1);
    if (linkUpNow != gLinkUp) {
        gLinkUp = linkUpNow;
        sendLinkEvent(gLinkUp);
    }
}

}  // namespace

void setup() {
    Serial.begin(115200);
    delay(250);

    Serial.println("XPB_DEBUG - minimal W5500 UDP sanity image");
    Serial.println("Pins: CS=A7(D21), RST=A6(D20), SPI D11/D12/D13");
    Serial.println("UDP: listen 8888, heartbeats to 8889, echo replies for CC RX only");

    parkSpiChipSelects();
    resetW5500();
    initEthernet();

    gUdpReady = (gUdp.begin(RtmNet::kUdpPort) == 1);
    gLinkUp = (Ethernet.link() == 1);

    Serial.print("XPBDBG;BOOT;UDP=");
    Serial.print(gUdpReady ? "OK" : "FAIL");
    Serial.print(";LINK=");
    Serial.println(gLinkUp ? "UP" : "DOWN");

    gLastHeartbeatMs = millis();
    gLastLinkPollMs = millis();
}

void loop() {
    pollLink();
    pumpRx();

    const uint32_t now = millis();
    if ((now - gLastHeartbeatMs) >= kHeartbeatMs) {
        gLastHeartbeatMs = now;
        sendHeartbeat();
    }
}
