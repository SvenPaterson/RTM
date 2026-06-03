#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include <utility/SdFat.h>
#include <Ethernet3.h>
#include <EthernetUdp3.h>

#include "LCDDriver.h"
#include "RtmNet.h"

namespace {
constexpr uint8_t kLcdCsPin = 8;  // Matches ExpansionBoard LCD_CS_ pin map.
constexpr uint8_t kSdCsPin = 4;   // Keep deselected while debugging TC path only.
constexpr uint8_t kTc1CsPin = 9;  // Keep MAX31855 TC1 deselected on shared SPI bus.
constexpr uint8_t kTc2CsPin = 10; // Keep MAX31855 TC2 deselected; doubles as AVR SS guard.
constexpr uint8_t kSpiMasterGuardPin = 10;  // Keep AVR SPI in master mode.
constexpr uint8_t kMisoPin = 12;
constexpr uint8_t kW5500CsPin = 21;   // A7, matches production XPB.
constexpr uint8_t kW5500RstPin = 20;  // A6, matches production XPB.
constexpr uint8_t kSdInitRateId = SPI_HALF_SPEED;  // Matches production SD.begin() path.
constexpr uint32_t kTcSpiHz = 4000000UL;           // Matches Max31855Min in production.
constexpr uint32_t kTickMs = 1000UL;
constexpr uint32_t kTempPollMs = 1000UL;
constexpr uint32_t kSdPollMs = 3000UL;
constexpr uint32_t kUdpPollMs = 50UL;
constexpr uint32_t kLinkPollMs = 500UL;
constexpr uint8_t kUdpMaxPacketsPerPoll = 3;
constexpr uint32_t kUdpBudgetUs = 2000UL;
constexpr uint32_t kCcHbStaleMs = 3000UL;
constexpr uint32_t kLateTickThresholdMs = 250UL;
constexpr uint32_t kSdInitRetryMs = 1000UL;
constexpr uint32_t kSdProbeMs = 12000UL;
constexpr uint32_t kSerialHeartbeatMs = 1000UL;
constexpr uint32_t kSerialBaud = 115200UL;

#if defined(LED_BUILTIN)
constexpr int8_t kHeartbeatLedPin = LED_BUILTIN;
#else
constexpr int8_t kHeartbeatLedPin = -1;
#endif

LCDDriver lcd(kLcdCsPin);
uint32_t lastTickMs = 0;
uint32_t lastTempPollMs = 0;
uint32_t lastSdPollMs = 0;
uint32_t lastUdpPollMs = 0;
uint32_t lastLinkPollMs = 0;
uint32_t lastSdProbeMs = 0;
uint32_t lastSdInitAttemptMs = 0;
uint32_t lastSerialHeartbeatMs = 0;
uint32_t lastLedMs = 0;
uint32_t ticks = 0;
bool ledOn = false;
bool sdReady = false;
uint32_t sdInitAttemptCount = 0;
bool netReady = false;
bool phyLinkUp = false;
uint32_t lastCcHbMs = 0;
uint32_t ccHbCount = 0;
uint32_t lastLoopStartUs = 0;
uint32_t loopDurationMaxUs = 0;
uint32_t lateTickCount = 0;
uint32_t netOverBudgetCount = 0;
uint8_t netPacketsLastPoll = 0;
uint32_t anyUdpCount = 0;
uint32_t nonCcUdpCount = 0;
uint32_t udpBindAttemptCount = 0;
uint32_t udpBindFailCount = 0;
uint8_t udpBindLastRc = 0;
bool firstUdpLogged = false;

enum class NetState : uint8_t {
    Down,
    Init,
    UpNoHb,
    UpHbOk,
    HbStale,
    RxOverrun,
};

enum class SpiState : uint8_t {
    Quiesced,
    Active,
    ContentionSuspect,
};

enum class LoopState : uint8_t {
    Healthy,
    LateTick,
    StarvedNet,
};

NetState netState = NetState::Down;
SpiState spiState = SpiState::Quiesced;
LoopState loopState = LoopState::Healthy;
uint32_t netStateTransitions = 0;
uint32_t loopStateTransitions = 0;
uint32_t spiStateTransitions = 0;

struct TcReading {
    float celsius = NAN;
    uint32_t raw = 0;
    bool fault = true;
    bool open = false;
    bool shortGnd = false;
    bool shortVcc = false;
};

TcReading tc1;
TcReading tc2;

enum class SdStatus : uint8_t {
    InitPending,
    CardInitFail,
    VolumeInitFail,
    RootOpenFail,
    CsvMissing,
    CsvOk,
};

SdStatus sdStatus = SdStatus::InitPending;
unsigned long csvSizeBytes = 0;
Sd2Card sdCard;
SdVolume sdVolume;
SdFile sdRoot;
EthernetUDP dbgUdp;

const char *sdStatusCode();

const char *netStateCode(NetState s) {
    switch (s) {
        case NetState::Down:
            return "DOWN";
        case NetState::Init:
            return "INIT";
        case NetState::UpNoHb:
            return "NOHB";
        case NetState::UpHbOk:
            return "HBOK";
        case NetState::HbStale:
            return "STAL";
        case NetState::RxOverrun:
            return "OVRN";
    }
    return "UNK";
}

const char *netUiCode(NetState s) {
    switch (s) {
        case NetState::Down:
            return "DWN";
        case NetState::Init:
            return "INI";
        case NetState::UpNoHb:
            return "NHB";
        case NetState::UpHbOk:
            return "OK";
        case NetState::HbStale:
            return "STL";
        case NetState::RxOverrun:
            return "OVR";
    }
    return "UNK";
}

const char *loopStateCode(LoopState s) {
    switch (s) {
        case LoopState::Healthy:
            return "OK";
        case LoopState::LateTick:
            return "LATE";
        case LoopState::StarvedNet:
            return "NSTARV";
    }
    return "UNK";
}

const char *loopUiCode(LoopState s) {
    switch (s) {
        case LoopState::Healthy:
            return "OK";
        case LoopState::LateTick:
            return "LT";
        case LoopState::StarvedNet:
            return "NS";
    }
    return "UK";
}

const char *spiStateCode(SpiState s) {
    switch (s) {
        case SpiState::Quiesced:
            return "QUIESCED";
        case SpiState::Active:
            return "ACTIVE";
        case SpiState::ContentionSuspect:
            return "CONTENTION";
    }
    return "UNK";
}

const char *sdUiCode() {
    switch (sdStatus) {
        case SdStatus::InitPending:
            return "INI";
        case SdStatus::CardInitFail:
            return "CRD";
        case SdStatus::VolumeInitFail:
            return "VOL";
        case SdStatus::RootOpenFail:
            return "ROT";
        case SdStatus::CsvMissing:
            return "CSV";
        case SdStatus::CsvOk:
            return "OK";
    }
    return "UNK";
}

void emitObserverEvent(const char *eventCode, const char *detail) {
    // Avoid socket send attempts while link is down; some W5500 states can
    // block long enough to look like a boot stall when the cable is absent.
    if (!netReady || !phyLinkUp) {
        return;
    }

    IPAddress obs(RtmNet::kObserverBroadcastIp[0],
                  RtmNet::kObserverBroadcastIp[1],
                  RtmNet::kObserverBroadcastIp[2],
                  RtmNet::kObserverBroadcastIp[3]);

    char msg[128];
    snprintf(msg, sizeof(msg),
             "DBG;EV=%s;TK=%lu;NET=%s;SD=%s;LOOP=%s;D=%s", 
             eventCode,
             static_cast<unsigned long>(ticks),
             netStateCode(netState),
             sdStatusCode(),
             loopStateCode(loopState),
             detail);

    dbgUdp.beginPacket(obs, RtmNet::kObserverPort);
    dbgUdp.write(reinterpret_cast<const uint8_t *>(msg), strlen(msg));
    dbgUdp.endPacket();
}

void setNetState(NetState next, const char *reason) {
    if (next == netState) {
        return;
    }
    netState = next;
    ++netStateTransitions;

    Serial.print("DBG_EVT NET->");
    Serial.print(netStateCode(netState));
    Serial.print(" reason=");
    Serial.println(reason);
    emitObserverEvent("NET", reason);
}

void setLoopState(LoopState next, const char *reason) {
    if (next == loopState) {
        return;
    }
    loopState = next;
    ++loopStateTransitions;

    Serial.print("DBG_EVT LOOP->");
    Serial.print(loopStateCode(loopState));
    Serial.print(" reason=");
    Serial.println(reason);
    emitObserverEvent("LOOP", reason);
}

void setSpiState(SpiState next, const char *reason) {
    if (next == spiState) {
        return;
    }

    const bool shouldEmit =
        (next == SpiState::ContentionSuspect) ||
        (spiState == SpiState::ContentionSuspect) ||
        (next == SpiState::Quiesced && strcmp(reason, "CS_PARKED") == 0);

    spiState = next;
    ++spiStateTransitions;

    if (shouldEmit) {
        Serial.print("DBG_EVT SPI->");
        Serial.print(spiStateCode(spiState));
        Serial.print(" reason=");
        Serial.println(reason);
        emitObserverEvent("SPI", reason);
    }
}

const char *linkCode() {
    return phyLinkUp ? "UP" : "DOWN";
}

bool bindUdpSocket(const char *reason) {
    ++udpBindAttemptCount;
    dbgUdp.stop();
    udpBindLastRc = (dbgUdp.begin(RtmNet::kUdpPort) == 1) ? 1 : 0;
    netReady = (udpBindLastRc == 1);
    if (!netReady) {
        ++udpBindFailCount;
    }

    Serial.print("DBG_EVT UDP->");
    Serial.print(netReady ? "BIND_OK" : "BIND_FAIL");
    Serial.print(" reason=");
    Serial.println(reason);
    return netReady;
}

void pollPhyLink() {
    const bool upNow = (Ethernet.link() == 1);
    if (upNow == phyLinkUp) {
        return;
    }

    phyLinkUp = upNow;
    if (phyLinkUp) {
        Serial.println("DBG_EVT PHY->UP reason=LINK_RESTORED");
        emitObserverEvent("PHY", "LINK_UP");
        bindUdpSocket("PHY_UP_REBIND");
        setNetState(NetState::UpNoHb, "PHY_UP");
    } else {
        Serial.println("DBG_EVT PHY->DOWN reason=LINK_LOST");
        dbgUdp.stop();
        netReady = false;
        setNetState(NetState::Down, "PHY_DOWN");
    }
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
    setSpiState(SpiState::Quiesced, "CS_PARKED");
}

void printBanner() {
    Serial.println();
    Serial.println("XPB_LCD_DEBUG TC+SD bench image");
    Serial.println("Tick cadence: 1000 ms (1 Hz)");
    Serial.println("TC poll cadence: 1000 ms");
    Serial.println("TC1 CS: D9 | TC2 CS: D10 | SD CS: D4 | LCD CS: D8");
    Serial.println("W5500 CS: D21(A7) RST: D20(A6) UDP: 8888");
    Serial.print("DBG_CFG IMG=XPB_LCD_DEBUG MOD=LCD,TC,SD,NET tickMs=");
    Serial.print(kTickMs);
    Serial.print(" udpMs=");
    Serial.print(kUdpPollMs);
    Serial.print(" udpMaxPkts=");
    Serial.print(kUdpMaxPacketsPerPoll);
    Serial.print(" udpBudgetUs=");
    Serial.println(kUdpBudgetUs);
}

bool isCcIp(const IPAddress &ip) {
    return ip[0] == RtmNet::kCcIp[0] &&
           ip[1] == RtmNet::kCcIp[1] &&
           ip[2] == RtmNet::kCcIp[2] &&
           ip[3] == RtmNet::kCcIp[3];
}

void bringUpW5500() {
    setNetState(NetState::Init, "W5500_RESET");

    pinMode(kW5500RstPin, OUTPUT);
    digitalWrite(kW5500RstPin, LOW);
    delay(10);
    digitalWrite(kW5500RstPin, HIGH);
    delay(100);

    pinMode(kW5500CsPin, OUTPUT);
    digitalWrite(kW5500CsPin, HIGH);

    Ethernet.setCsPin(kW5500CsPin);
    uint8_t mac[6];
    for (uint8_t i = 0; i < 6; ++i) {
        mac[i] = RtmNet::kXpbMac[i];
    }
    IPAddress ip(RtmNet::kXpbIp[0], RtmNet::kXpbIp[1], RtmNet::kXpbIp[2], RtmNet::kXpbIp[3]);
    Ethernet.begin(mac, ip);
    phyLinkUp = (Ethernet.link() == 1);
    bindUdpSocket("BOOT_INIT");
    if (netReady && phyLinkUp) {
        setNetState(NetState::UpNoHb, "UDP_LISTEN");
    } else if (netReady && !phyLinkUp) {
        setNetState(NetState::Down, "PHY_DOWN");
    } else {
        setNetState(NetState::Down, "UDP_BIND_FAIL");
    }

    Serial.print("NET up at ");
    Serial.print(ip[0]);
    Serial.print('.');
    Serial.print(ip[1]);
    Serial.print('.');
    Serial.print(ip[2]);
    Serial.print('.');
    Serial.println(ip[3]);
    Serial.print("NET link=");
    Serial.println(linkCode());
}

void pollCcHeartbeats() {
    if (!netReady) {
        setNetState(NetState::Down, "NET_NOT_READY");
        return;
    }
    if (!phyLinkUp) {
        setNetState(NetState::Down, "PHY_DOWN");
        return;
    }

    // Budget UDP processing so high traffic cannot starve the rest of loop().
    uint8_t packetsProcessed = 0;
    const uint32_t pollStartUs = micros();
    bool budgetHit = false;
    while (packetsProcessed < kUdpMaxPacketsPerPoll) {
        if ((micros() - pollStartUs) >= kUdpBudgetUs) {
            budgetHit = true;
            break;
        }

        const int sz = dbgUdp.parsePacket();
        if (sz <= 0) {
            break;
        }
        ++packetsProcessed;
        ++anyUdpCount;

        IPAddress src = dbgUdp.remoteIP();
        const bool fromCc = isCcIp(src);

        uint8_t pkt[192];
        int n = dbgUdp.read(pkt, sizeof(pkt) - 1);
        if (n < 0) {
            n = 0;
        }
        pkt[n] = 0;

        while (dbgUdp.available() > 0) {
            dbgUdp.read();
        }

        if (!firstUdpLogged) {
            firstUdpLogged = true;
            Serial.print("DBG_EVT UDP_FIRST src=");
            Serial.print(src[0]);
            Serial.print('.');
            Serial.print(src[1]);
            Serial.print('.');
            Serial.print(src[2]);
            Serial.print('.');
            Serial.print(src[3]);
            Serial.print(" bytes=");
            Serial.println(n);
        }

        if (fromCc && strstr(reinterpret_cast<const char *>(pkt), "HB;") != nullptr) {
            lastCcHbMs = millis();
            ++ccHbCount;
        } else if (!fromCc) {
            ++nonCcUdpCount;
        }
    }

    netPacketsLastPoll = packetsProcessed;
    if (budgetHit) {
        ++netOverBudgetCount;
        setNetState(NetState::RxOverrun, "UDP_BUDGET");
    } else {
        if (lastCcHbMs == 0) {
            setNetState(NetState::UpNoHb, "WAIT_HB");
        } else if ((millis() - lastCcHbMs) > kCcHbStaleMs) {
            setNetState(NetState::HbStale, "HB_STALE");
        } else {
            setNetState(NetState::UpHbOk, "HB_OK");
        }
    }
}

const char *ccHbStatusCode() {
    if (!netReady) {
        return "NETDOWN";
    }
    if (lastCcHbMs == 0) {
        return "NOHB";
    }
    if (millis() - lastCcHbMs > kCcHbStaleMs) {
        return "STALE";
    }
    return "OK";
}

const char *ccHbUiCode() {
    if (!netReady) {
        return "NUP";
    }
    if (lastCcHbMs == 0) {
        return "NOHB";
    }
    if (millis() - lastCcHbMs > kCcHbStaleMs) {
        return "STAL";
    }
    return "OK";
}

const char *sdStatusCode() {
    switch (sdStatus) {
        case SdStatus::InitPending:
            return "INIT";
        case SdStatus::CardInitFail:
            return "CARDFAIL";
        case SdStatus::VolumeInitFail:
            return "VOLFAIL";
        case SdStatus::RootOpenFail:
            return "ROOTFAIL";
        case SdStatus::CsvMissing:
            return "CSVMISS";
        case SdStatus::CsvOk:
            return "OK";
    }
    return "UNK";
}

const char *sdCardErrName(uint8_t code) {
    switch (code) {
        case SD_CARD_ERROR_CMD0:
            return "CMD0";
        case SD_CARD_ERROR_CMD8:
            return "CMD8";
        case SD_CARD_ERROR_ACMD41:
            return "ACMD41";
        case SD_CARD_ERROR_CMD58:
            return "CMD58";
        default:
            return "OTHER";
    }
}

bool trySdInitWithRetry() {
    parkSpiChipSelects();
    delay(10);
    for (uint8_t i = 0; i < 10; ++i) {
        SPI.transfer(0xFF);
    }

    ++sdInitAttemptCount;
    Serial.print("SD init attempt #");
    Serial.print(static_cast<unsigned long>(sdInitAttemptCount));
    Serial.println();

    if (!sdCard.init(kSdInitRateId, kSdCsPin)) {
        sdStatus = SdStatus::CardInitFail;
        const uint8_t err = sdCard.errorCode();
        const uint8_t data = sdCard.errorData();
        Serial.print("  stage: card.init FAIL err=");
        Serial.print(sdCardErrName(err));
        Serial.print("(0x");
        Serial.print(err, HEX);
        Serial.print(") data=0x");
        Serial.println(data, HEX);
        return false;
    }
    if (!sdVolume.init(sdCard)) {
        sdStatus = SdStatus::VolumeInitFail;
        Serial.println("  stage: volume.init FAIL (format FAT16/32)");
        return false;
    }
    if (!sdRoot.openRoot(sdVolume)) {
        sdStatus = SdStatus::RootOpenFail;
        Serial.println("  stage: openRoot FAIL");
        return false;
    }

    Serial.println("  stage: card/volume/root OK");
    return true;
}

void maybePrintSdWiringProbe() {
    const uint32_t now = millis();
    if (now - lastSdProbeMs < kSdProbeMs) {
        return;
    }
    lastSdProbeMs = now;

    pinMode(kMisoPin, INPUT_PULLUP);
    Serial.print("SD probe: MISO D12 idle=");
    Serial.println(digitalRead(kMisoPin));
    Serial.println("SD probe: D4-only path (CS assumed correct)");

    // Raw CMD0 probe: if every byte is 0xFF, the card is not responding on MISO.
    parkSpiChipSelects();
    SPI.beginTransaction(SPISettings(250000UL, MSBFIRST, SPI_MODE0));

    // >=74 clocks with CS high.
    for (uint8_t i = 0; i < 10; ++i) {
        SPI.transfer(0xFF);
    }

    digitalWrite(kSdCsPin, LOW);
    SPI.transfer(0x40);  // CMD0
    SPI.transfer(0x00);
    SPI.transfer(0x00);
    SPI.transfer(0x00);
    SPI.transfer(0x00);
    SPI.transfer(0x95);  // valid CRC for CMD0 in SPI mode

    uint8_t firstResp = 0xFF;
    uint8_t firstRespIdx = 0xFF;
    Serial.print("SD probe: CMD0 resp bytes=");
    for (uint8_t i = 0; i < 10; ++i) {
        const uint8_t b = SPI.transfer(0xFF);
        if (i) {
            Serial.print(',');
        }
        Serial.print("0x");
        if (b < 16) Serial.print('0');
        Serial.print(b, HEX);

        if (firstResp == 0xFF && b != 0xFF) {
            firstResp = b;
            firstRespIdx = i;
        }
    }
    Serial.println();

    digitalWrite(kSdCsPin, HIGH);
    SPI.transfer(0xFF);
    SPI.endTransaction();

    if (firstResp == 0xFF) {
        Serial.println("SD probe: CMD0 no response (all 0xFF)");
    } else {
        Serial.print("SD probe: CMD0 first non-FF at idx ");
        Serial.print(firstRespIdx);
        Serial.print(" value=0x");
        if (firstResp < 16) Serial.print('0');
        Serial.println(firstResp, HEX);
    }
}

void refreshSdStatus() {
    csvSizeBytes = 0;

    if (!sdReady) {
        const uint32_t now = millis();
        if (now - lastSdInitAttemptMs < kSdInitRetryMs) {
            if (sdStatus == SdStatus::InitPending) {
                sdStatus = SdStatus::CardInitFail;
            }
            return;
        }

        lastSdInitAttemptMs = now;
        sdReady = trySdInitWithRetry();
        if (!sdReady) {
            maybePrintSdWiringProbe();
            return;
        }
    }

    if (!sdRoot.isOpen() && !sdRoot.openRoot(sdVolume)) {
        sdStatus = SdStatus::RootOpenFail;
        return;
    }

    SdFile csv;
    if (!csv.open(&sdRoot, "protocol.csv", O_READ)) {
        sdStatus = SdStatus::CsvMissing;
        return;
    }

    csvSizeBytes = static_cast<unsigned long>(csv.fileSize());
    csv.close();

    if (csvSizeBytes == 0) {
        sdStatus = SdStatus::CsvMissing;
    } else {
        sdStatus = SdStatus::CsvOk;
    }
}

TcReading readMax31855(uint8_t csPin) {
    TcReading out;

    setSpiState(SpiState::Active, "TC_READ");

    // Keep non-selected SPI devices deselected before each transfer.
    digitalWrite(kSdCsPin, HIGH);
    digitalWrite(kLcdCsPin, HIGH);
    digitalWrite(kTc1CsPin, HIGH);
    digitalWrite(kTc2CsPin, HIGH);

    digitalWrite(csPin, LOW);
    SPI.beginTransaction(SPISettings(kTcSpiHz, MSBFIRST, SPI_MODE0));
    uint32_t raw = 0;
    for (uint8_t i = 0; i < 4; ++i) {
        raw = (raw << 8) | static_cast<uint32_t>(SPI.transfer(0x00));
    }
    SPI.endTransaction();
    digitalWrite(csPin, HIGH);

    out.raw = raw;
    out.fault = (raw & 0x00010000UL) != 0;
    out.open = (raw & 0x1U) != 0;
    out.shortGnd = (raw & 0x2U) != 0;
    out.shortVcc = (raw & 0x4U) != 0;

    if (!out.fault) {
        int16_t signedTemp = static_cast<int16_t>(raw >> 16);
        signedTemp >>= 2;
        out.celsius = static_cast<float>(signedTemp) * 0.25f;
    }

    return out;
}

void readThermocouples() {
    tc1 = readMax31855(kTc1CsPin);
    tc2 = readMax31855(kTc2CsPin);
}

void printFaultFlags(const TcReading &tc) {
    if (!tc.fault) {
        Serial.print("none");
        return;
    }

    bool any = false;
    if (tc.open) {
        Serial.print("OC");
        any = true;
    }
    if (tc.shortGnd) {
        if (any) Serial.print("|");
        Serial.print("SCG");
        any = true;
    }
    if (tc.shortVcc) {
        if (any) Serial.print("|");
        Serial.print("SCV");
        any = true;
    }
    if (!any) {
        Serial.print("FAULT_NO_SUBCODE");
    }
}

void printHeartbeat() {
    Serial.print("HB tick=");
    Serial.print(static_cast<unsigned long>(ticks));
    Serial.print(" tc1=");
    if (tc1.fault) {
        Serial.print("FAULT(");
        printFaultFlags(tc1);
        Serial.print(") raw=0x");
        Serial.print(tc1.raw, HEX);
    } else {
        Serial.print(tc1.celsius, 2);
        Serial.print("C");
    }
    Serial.print(" tc2=");
    if (tc2.fault) {
        Serial.print("FAULT(");
        printFaultFlags(tc2);
        Serial.print(") raw=0x");
        Serial.print(tc2.raw, HEX);
    } else {
        Serial.print(tc2.celsius, 2);
        Serial.print("C");
    }

    Serial.print(" sd=");
    Serial.print(sdStatusCode());
    if (sdStatus == SdStatus::CsvOk) {
        Serial.print(" ");
        Serial.print(csvSizeBytes);
        Serial.print("B");
    }

    Serial.print(" hb=");
    Serial.print(ccHbStatusCode());
    Serial.print(" cnt=");
    Serial.print(ccHbCount);
    if (lastCcHbMs != 0) {
        Serial.print(" ageMs=");
        Serial.print(millis() - lastCcHbMs);
    }
    Serial.println();

    const uint32_t hbAgeMs = (lastCcHbMs == 0) ? 0xFFFFFFFFUL : (millis() - lastCcHbMs);
    Serial.print("DBG;TK=");
    Serial.print(static_cast<unsigned long>(ticks));
    Serial.print(";NET=");
    Serial.print(netStateCode(netState));
    Serial.print(";LINK=");
    Serial.print(linkCode());
    Serial.print(";HB_CNT=");
    Serial.print(ccHbCount);
    Serial.print(";HB_AGE=");
    if (hbAgeMs == 0xFFFFFFFFUL) {
        Serial.print("NONE");
    } else {
        Serial.print(hbAgeMs);
    }
    Serial.print(";UDP_PP=");
    Serial.print(netPacketsLastPoll);
    Serial.print(";UDP_ANY=");
    Serial.print(anyUdpCount);
    Serial.print(";UDP_NONCC=");
    Serial.print(nonCcUdpCount);
    Serial.print(";UDP_OB=");
    Serial.print(netOverBudgetCount);
    Serial.print(";UDP_BA=");
    Serial.print(udpBindAttemptCount);
    Serial.print(";UDP_BF=");
    Serial.print(udpBindFailCount);
    Serial.print(";UDP_BR=");
    Serial.print(udpBindLastRc);
    Serial.print(";SD=");
    Serial.print(sdStatusCode());
    Serial.print(";SPI=");
    Serial.print(spiStateCode(spiState));
    Serial.print(";LOOP=");
    Serial.print(loopStateCode(loopState));
    Serial.print(";LATE=");
    Serial.print(lateTickCount);
    Serial.print(";LUS_MAX=");
    Serial.print(loopDurationMaxUs);
    Serial.print(";NTR=");
    Serial.print(netStateTransitions);
    Serial.print(";LTR=");
    Serial.print(loopStateTransitions);
    Serial.print(";STR=");
    Serial.println(spiStateTransitions);
}

void formatCelsius(char *dst, size_t len, float celsius) {
    if (len == 0) {
        return;
    }

    const int32_t scaled = static_cast<int32_t>(celsius * 100.0f + (celsius >= 0.0f ? 0.5f : -0.5f));
    const int32_t absScaled = (scaled < 0) ? -scaled : scaled;
    const int32_t whole = absScaled / 100;
    const int32_t frac = absScaled % 100;

    if (scaled < 0) {
        snprintf(dst, len, "-%ld.%02ld", static_cast<long>(whole), static_cast<long>(frac));
    } else {
        snprintf(dst, len, "%ld.%02ld", static_cast<long>(whole), static_cast<long>(frac));
    }
}

void refreshLcdTemps() {
    char line1[21];
    char line2[21];
    char line3[21];
    char tcBuf[12];

    if (tc1.fault) {
        if (tc1.open) {
            snprintf(line1, sizeof(line1), "TC1(D9): FAULT OC");
        } else if (tc1.shortGnd) {
            snprintf(line1, sizeof(line1), "TC1(D9): FAULT SCG");
        } else if (tc1.shortVcc) {
            snprintf(line1, sizeof(line1), "TC1(D9): FAULT SCV");
        } else {
            snprintf(line1, sizeof(line1), "TC1(D9): FAULT");
        }
    } else {
        formatCelsius(tcBuf, sizeof(tcBuf), tc1.celsius);
        snprintf(line1, sizeof(line1), "TC1(D9): %s C", tcBuf);
    }

    if (tc2.fault) {
        if (tc2.open) {
            snprintf(line2, sizeof(line2), "TC2(D10): FAULT OC");
        } else if (tc2.shortGnd) {
            snprintf(line2, sizeof(line2), "TC2(D10): FAULT SCG");
        } else if (tc2.shortVcc) {
            snprintf(line2, sizeof(line2), "TC2(D10): FAULT SCV");
        } else {
            snprintf(line2, sizeof(line2), "TC2(D10): FAULT");
        }
    } else {
        formatCelsius(tcBuf, sizeof(tcBuf), tc2.celsius);
        snprintf(line2, sizeof(line2), "TC2(D10): %s C", tcBuf);
    }

    snprintf(line3,
             sizeof(line3),
             "Tk%lu S:%s N:%s L:%s",
             static_cast<unsigned long>(ticks),
             sdUiCode(),
             netUiCode(netState),
             loopUiCode(loopState));

    lcd.setLineCenter(0, "XPB TC+SD+NET");
    lcd.setLineLeft(1, line1);
    lcd.setLineLeft(2, line2);
    lcd.setLineLeft(3, line3);
}

void refreshHeartbeatLed(uint32_t now) {
    if (kHeartbeatLedPin < 0 || now - lastLedMs < kTickMs / 2) {
        return;
    }

    lastLedMs = now;
    ledOn = !ledOn;
    digitalWrite(kHeartbeatLedPin, ledOn ? HIGH : LOW);
}
}  // namespace

void setup() {
    Serial.begin(kSerialBaud);
    delay(300);
    printBanner();

    if (kHeartbeatLedPin >= 0) {
        pinMode(kHeartbeatLedPin, OUTPUT);
        digitalWrite(kHeartbeatLedPin, LOW);
    }

    parkSpiChipSelects();
    bringUpW5500();

    lcd.begin();
    lcd.clearScreen();
    readThermocouples();
    refreshSdStatus();
    refreshLcdTemps();
    lastTickMs = millis();
    printHeartbeat();
    lcd.flush();
}

void loop() {
    const uint32_t loopStartUs = micros();
    lastLoopStartUs = loopStartUs;

    const uint32_t now = millis();
    bool updatedTemps = false;
    bool updatedSd = false;
    bool updatedNet = false;

    refreshHeartbeatLed(now);

    if (now - lastUdpPollMs >= kUdpPollMs) {
        lastUdpPollMs = now;
        const uint32_t hbBefore = ccHbCount;
        pollCcHeartbeats();
        updatedNet = (ccHbCount != hbBefore);
    }

    if (now - lastLinkPollMs >= kLinkPollMs) {
        lastLinkPollMs = now;
        pollPhyLink();
    }

    if (now - lastSdPollMs >= kSdPollMs) {
        lastSdPollMs = now;
        refreshSdStatus();
        updatedSd = true;
    }

    // Give SD init attempts priority on shared SPI bus cycles.
    if (!updatedSd && now - lastTempPollMs >= kTempPollMs) {
        lastTempPollMs = now;
        readThermocouples();
        setSpiState(SpiState::Quiesced, "TC_DONE");
        updatedTemps = true;
    }

    if (updatedTemps || updatedSd || updatedNet) {
        refreshLcdTemps();
        if (updatedSd || (now - lastSerialHeartbeatMs >= kSerialHeartbeatMs)) {
            lastSerialHeartbeatMs = now;
            printHeartbeat();
        }
    }

    if (now - lastTickMs < kTickMs) {
        lcd.flush();
        const uint32_t loopUs = micros() - loopStartUs;
        if (loopUs <= 2000000UL && loopUs > loopDurationMaxUs) {
            loopDurationMaxUs = loopUs;
        }
        return;
    }

    const uint32_t tickDeltaMs = now - lastTickMs;
    if (lastTickMs != 0 && tickDeltaMs > (kTickMs + kLateTickThresholdMs)) {
        ++lateTickCount;
        setLoopState(LoopState::LateTick, "TICK_LAG");
    } else if (netState == NetState::RxOverrun) {
        setLoopState(LoopState::StarvedNet, "NET_OVERRUN");
    } else {
        setLoopState(LoopState::Healthy, "TICK_OK");
    }

    lastTickMs = now;
    ++ticks;
    refreshLcdTemps();
    lcd.flush();

    const uint32_t loopUs = micros() - loopStartUs;
    if (loopUs <= 2000000UL && loopUs > loopDurationMaxUs) {
        loopDurationMaxUs = loopUs;
    }
}
