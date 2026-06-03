// cc_debug.cpp — ClearCore minimal sender-side ground-truth debug image
//
// Purpose: provide independent, deterministic visibility into whether CC is
// actually emitting UDP frames on the wire. The production CC firmware is
// silent on its own TX behavior (no TX result counters surfaced to serial),
// so when XPB receives nothing we cannot tell from XPB telemetry alone
// whether the gap is sender, wire, or receiver.
//
// What this image does, and only this:
//   * Bring up ClearCore Ethernet with kCcMac / kCcIp (production parity).
//   * Open a single EthernetUDP socket bound to kUdpPort (8888).
//   * Every 250 ms:
//       - unicast a fixed CCDBG payload to kXpbIp:kUdpPort (production path)
//       - broadcast-tee the same payload to 10.0.0.255:8890
//         (separate from kObserverPort 8889 so observer parsers do not
//          have to special-case debug frames)
//   * Record beginPacket / endPacket return codes per send. beginPacket==0
//     normally indicates ARP-resolve failure for the destination IP, so it
//     is the canonical "wire/topology" signal from the sender side.
//   * Passively parse inbound on :8888, count any frames, count frames
//     whose source IP matches kXpbIp (so we can verify bidirectional
//     reachability if the operator runs the XPB debug image alongside).
//   * Emit a structured CCDBG;TK=...;... serial line once per second.
//
// What this image does NOT do:
//   * No motor, no PID, no heater, no SD, no protocol, no RtmComms framing.
//   * No production state machine. This is a wire-level probe only.
//
// Operator gates (see /memories/session/plan.md Phase G):
//   G3  PC udp_capture on :8890 sees CCDBG frames           -> CC TX is alive
//   G1  XPB debug image UDP_ANY/HB_CNT advance in lockstep  -> wire OK end-to-end
//   G2  Power XPB off -> bpFail (ARP fail) climbs on CC     -> ARP path honest

#include "ClearCore.h"
#include "ClearCoreElapsedMillis.h"
#include "RtmNet.h"

#include <Ethernet.h>

#include <stdio.h>
#include <stdint.h>
#include <string.h>

// Local aliases for ClearCore connectors. Mirrors ClearCoreRTM.h so this
// image stays independent of the production runtime header.
#define SerialPort ConnectorUsb
#define LED_PIN    ConnectorIO0

namespace {

// --- Cadences ---
constexpr uint32_t kHbCadenceMs        = 250;   // mirrors production CC HB cadence
constexpr uint32_t kTelemetryCadenceMs = 1000;  // mirrors XPB debug DBG line cadence
constexpr uint16_t kBroadcastTeePort   = 8890;  // distinct from kObserverPort (8889)

// --- TX bookkeeping ---
uint32_t hbSeq         = 0;
uint32_t txAttempts    = 0;
uint32_t txOk          = 0;   // both beginPacket and endPacket non-zero
uint32_t bpFail        = 0;   // beginPacket returned 0 (commonly ARP fail)
uint32_t epFail        = 0;   // endPacket returned 0 (link/PHY/driver issue)
uint32_t teeAttempts   = 0;
uint32_t teeOk         = 0;
uint32_t teeFail       = 0;

// --- RX bookkeeping ---
uint32_t rxAny         = 0;   // any datagram parsed on :8888
uint32_t rxFromXpb     = 0;   // src IP == kXpbIp
uint32_t rxNonXpb      = 0;

// --- Loop health ---
uint32_t loopMaxUs     = 0;
uint32_t tickCount     = 0;

// --- Last-seen return codes for serial trace ---
uint8_t  lastBpRc      = 0;
uint8_t  lastEpRc      = 0;

EthernetUDP gUdp;

IPAddress xpbIp(RtmNet::kXpbIp[0],   RtmNet::kXpbIp[1],
                RtmNet::kXpbIp[2],   RtmNet::kXpbIp[3]);
IPAddress bcastIp(RtmNet::kObserverBroadcastIp[0],
                  RtmNet::kObserverBroadcastIp[1],
                  RtmNet::kObserverBroadcastIp[2],
                  RtmNet::kObserverBroadcastIp[3]);

void dbgWrite(const char *s) {
    while (*s) {
        SerialPort.SendChar(*s++);
    }
}

void dbgWriteln(const char *s) {
    dbgWrite(s);
    SerialPort.SendChar('\n');
}

bool isXpbIp(const IPAddress &ip) {
    return ip[0] == RtmNet::kXpbIp[0] && ip[1] == RtmNet::kXpbIp[1] &&
           ip[2] == RtmNet::kXpbIp[2] && ip[3] == RtmNet::kXpbIp[3];
}

void sendHeartbeat() {
    char payload[80];
    const int len = snprintf(payload, sizeof(payload),
                             "CCDBG;SEQ=%lu;UP_MS=%lu\n",
                             (unsigned long)hbSeq,
                             (unsigned long)Milliseconds());
    if (len <= 0) return;
    const size_t plen = (size_t)len;

    // --- Unicast to XPB (production path) ---
    ++txAttempts;
    const uint8_t bp = gUdp.beginPacket(xpbIp, RtmNet::kUdpPort);
    lastBpRc = bp;
    if (!bp) {
        ++bpFail;
    } else {
        gUdp.write((const uint8_t*)payload, plen);
        const uint8_t ep = gUdp.endPacket();
        lastEpRc = ep;
        if (!ep) {
            ++epFail;
        } else {
            ++txOk;
        }
    }

    // --- Broadcast tee (PC observer ground truth) ---
    ++teeAttempts;
    const uint8_t bpTee = gUdp.beginPacket(bcastIp, kBroadcastTeePort);
    if (!bpTee) {
        ++teeFail;
    } else {
        gUdp.write((const uint8_t*)payload, plen);
        const uint8_t epTee = gUdp.endPacket();
        if (!epTee) ++teeFail; else ++teeOk;
    }

    ++hbSeq;
}

void pumpRx() {
    int sz = gUdp.parsePacket();
    while (sz > 0) {
        ++rxAny;
        const IPAddress src = gUdp.remoteIP();
        if (isXpbIp(src)) ++rxFromXpb; else ++rxNonXpb;
        // Drain payload so the socket buffer doesn't stall.
        while (sz-- > 0) (void)gUdp.read();
        sz = gUdp.parsePacket();
    }
}

void emitTelemetry() {
    char line[192];
    snprintf(line, sizeof(line),
             "CCDBG;TK=%lu;HB_SEQ=%lu;TX_OK=%lu;BP_FAIL=%lu;EP_FAIL=%lu;"
             "TEE_OK=%lu;TEE_FAIL=%lu;BP_RC=%u;EP_RC=%u;"
             "RX_ANY=%lu;RX_XPB=%lu;RX_OTHER=%lu;LUS_MAX=%lu\n",
             (unsigned long)tickCount,
             (unsigned long)hbSeq,
             (unsigned long)txOk,
             (unsigned long)bpFail,
             (unsigned long)epFail,
             (unsigned long)teeOk,
             (unsigned long)teeFail,
             (unsigned)lastBpRc,
             (unsigned)lastEpRc,
             (unsigned long)rxAny,
             (unsigned long)rxFromXpb,
             (unsigned long)rxNonXpb,
             (unsigned long)loopMaxUs);
    dbgWrite(line);
    // Reset the loop watermark each second so we see worst-case per window.
    loopMaxUs = 0;
}

}  // namespace

int main() {
    // --- USB serial (ClearCore CDC) ---
    SerialPort.Mode(Connector::USB_CDC);
    SerialPort.Speed(9600);
    SerialPort.PortOpen();
    const uint32_t tSer0 = Milliseconds();
    while (!SerialPort && (Milliseconds() - tSer0) < 5000) { /* wait briefly */ }

    dbgWriteln("");
    dbgWriteln("CCDBG image — minimal CC sender-side ground truth");
    dbgWriteln("Cadence: HB 250ms unicast 10.0.0.11:8888, tee 10.0.0.255:8890");
    dbgWriteln("Telemetry: CCDBG;... once per second");

    // --- Onboard LED on so operator can see image is alive ---
    LED_PIN.Mode(Connector::OUTPUT_DIGITAL);
    LED_PIN.State(true);

    // --- Ethernet bring-up (mirror production CC parity) ---
    uint8_t mac[6];
    for (uint8_t i = 0; i < 6; ++i) mac[i] = RtmNet::kCcMac[i];
    IPAddress ccIp(RtmNet::kCcIp[0], RtmNet::kCcIp[1],
                   RtmNet::kCcIp[2], RtmNet::kCcIp[3]);
    Ethernet.begin(mac, ccIp);

    if (!gUdp.begin(RtmNet::kUdpPort)) {
        dbgWriteln("CCDBG FATAL: gUdp.begin() failed");
        // Fall through; further sends will fail loudly via counters.
    } else {
        dbgWriteln("CCDBG: UDP socket bound on :8888");
    }

    elapsedMillis hbTmr     = 0;
    elapsedMillis telTmr    = 0;

    while (true) {
        const uint32_t loopT0 = Microseconds();

        // RX drain first so we never block sends on a backlog.
        pumpRx();

        if (hbTmr >= kHbCadenceMs) {
            hbTmr = 0;
            sendHeartbeat();
        }

        if (telTmr >= kTelemetryCadenceMs) {
            telTmr = 0;
            ++tickCount;
            emitTelemetry();
        }

        const uint32_t loopUs = Microseconds() - loopT0;
        if (loopUs > loopMaxUs) loopMaxUs = loopUs;
    }
}
