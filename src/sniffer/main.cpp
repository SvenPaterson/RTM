#include <Arduino.h>

#ifndef OUTPUT_OPENDRAIN
#define OUTPUT_OPENDRAIN OUTPUT
#endif

namespace {
constexpr uint32_t kTtlBaud = 9600;
constexpr uint32_t kUsbBaud = 460800;
constexpr size_t kLineBufferSize = 256;
constexpr uint32_t kSyncTimeoutMs = 5000;
constexpr bool kEnableHexBootWindow = false;
constexpr uint32_t kHexBootWindowMs = 2000;
constexpr bool kForwardRawBytes = false;
constexpr uint8_t kNonPrintableWarningThreshold = 4;
constexpr uint8_t kRunCtrlPin = 14;
constexpr uint8_t kResetCtrlPin = 15;
constexpr uint8_t kPowerCtrlPin = 22;
constexpr size_t kUsbCommandMaxLen = 80;

// Onboard LED blink protocol
constexpr uint32_t kStandbyBlinkMs = 500;  // slow blink: waiting for host connect
constexpr uint32_t kActiveBlinkMs  = 75;  // fast blink: actively sniffing
constexpr uint32_t kIdleTimeoutMs  = 2000; // switch to slow if no traffic

// Teensy 4.0 wiring:
//   Pin 7 (Serial2 RX) taps CC's TX wire   → logs as CC_TX
//   Pin 0 (Serial1 RX) taps XPB's TX wire  → logs as XPB_TX
constexpr uint8_t kCcTxRxPin  = 7;   // Serial2 — listens to ClearCore transmissions
constexpr uint8_t kXpbTxRxPin = 0;   // Serial1 — listens to Expansion Board transmissions

const char kHexDigits[] = "0123456789ABCDEF";

uint32_t gBootStartMs = 0;

// LED blink state
uint32_t gLedLastToggleMs = 0;
uint32_t gLedIntervalMs = kStandbyBlinkMs;
bool gLedOn = false;
uint32_t gLastActivityMs = 0;  // last time we saw any UART byte
bool gRunLatched = false;
bool gResetLatched = false;
bool gRunPulseActive = false;
bool gResetPulseActive = false;
uint32_t gRunPulseReleaseMs = 0;
uint32_t gResetPulseReleaseMs = 0;
bool gRunAsserted = false;
bool gResetAsserted = false;
bool gPowerAsserted = false;
String gUsbLine;

void UpdateLedBlink() {
    const uint32_t now = millis();
    if (now - gLedLastToggleMs >= gLedIntervalMs) {
        gLedLastToggleMs = now;
        gLedOn = !gLedOn;
        digitalWrite(LED_BUILTIN, gLedOn ? HIGH : LOW);
    }
}

struct ChannelState {
    ChannelState(HardwareSerial &serial_ref, const char *tag_name)
        : serial(serial_ref), tag(tag_name) {}

    HardwareSerial &serial;
    const char *tag;
    uint8_t buffer[kLineBufferSize] = {};
    size_t length = 0;
    bool firstLinePending = true;
    bool warnedMismatch = false;
    bool syncedNewline = false;
    uint32_t lastStampMs = 0;
};

ChannelState gCcChannel(Serial2, "CC_TX");    // pin 7 — ClearCore is transmitting
ChannelState gXpbChannel(Serial1, "XPB_TX");   // pin 0 — Expansion Board is transmitting

bool IsPrintableAscii(uint8_t value) {
    return (value >= 0x20 && value <= 0x7E) || value == '\t';
}

void UpdateRunOutput() {
    const bool assertLine = gRunLatched || gRunPulseActive;
    digitalWrite(kRunCtrlPin, assertLine ? LOW : HIGH);
    gRunAsserted = assertLine;
}

void UpdateResetOutput() {
    const bool assertLine = gResetLatched || gResetPulseActive;
    digitalWrite(kResetCtrlPin, assertLine ? LOW : HIGH);
    gResetAsserted = assertLine;
}

void UpdatePowerOutput() {
    digitalWrite(kPowerCtrlPin, gPowerAsserted ? HIGH : LOW);
}

void PrintLineStatus(const char *label, bool asserted, bool latched, bool pulsing) {
    Serial.printf("[CTRL] %s line %s (%s)\n",
                  label,
                  asserted ? "LOW" : "HIGH",
                  latched ? "latched" : (pulsing ? "pulse" : "idle"));
}

void EmitControlStatus() {
    PrintLineStatus("RUN", gRunAsserted, gRunLatched, gRunPulseActive);
    PrintLineStatus("RESET", gResetAsserted, gResetLatched, gResetPulseActive);
    Serial.printf("[CTRL] POWER %s\n", gPowerAsserted ? "ON" : "OFF");
}

bool ParseActiveToken(String token, bool &active) {
    token.trim();
    token.toUpperCase();
    if (token == "1" || token == "ON" || token == "LOW" || token == "ASSERT" || token == "PRESS") {
        active = true;
        return true;
    }
    if (token == "0" || token == "OFF" || token == "HIGH" || token == "RELEASE" || token == "OPEN") {
        active = false;
        return true;
    }
    return false;
}

void HandleLatchedCommand(bool isRun, bool active) {
    if (isRun) {
        if (active && (gResetLatched || gResetPulseActive)) {
            Serial.println("[CTRL] RUN request denied; RESET already asserted");
            return;
        }
        gRunLatched = active;
        if (!active) {
            gRunPulseActive = false;
        }
        UpdateRunOutput();
        PrintLineStatus("RUN", gRunAsserted, gRunLatched, gRunPulseActive);
    } else {
        if (active && (gRunLatched || gRunPulseActive)) {
            Serial.println("[CTRL] RESET request denied; RUN already asserted");
            return;
        }
        gResetLatched = active;
        if (!active) {
            gResetPulseActive = false;
        }
        UpdateResetOutput();
        PrintLineStatus("RESET", gResetAsserted, gResetLatched, gResetPulseActive);
    }
}

void HandlePowerCommand(bool active) {
    gPowerAsserted = active;
    UpdatePowerOutput();
    Serial.printf("[CTRL] POWER %s\n", gPowerAsserted ? "ON" : "OFF");
}

void HandlePulseCommand(bool isRun, uint32_t durationMs) {
    const uint32_t deadline = millis() + durationMs;
    if (isRun) {
        if (gResetLatched || gResetPulseActive) {
            Serial.println("[CTRL] RUN pulse denied; RESET already asserted");
            return;
        }
        gRunPulseActive = true;
        gRunPulseReleaseMs = deadline;
        UpdateRunOutput();
        Serial.printf("[CTRL] RUN pulse %lu ms%s\n", static_cast<unsigned long>(durationMs),
                      gRunLatched ? " (latched hold will keep line LOW)" : "");
    } else {
        if (gRunLatched || gRunPulseActive) {
            Serial.println("[CTRL] RESET pulse denied; RUN already asserted");
            return;
        }
        gResetPulseActive = true;
        gResetPulseReleaseMs = deadline;
        UpdateResetOutput();
        Serial.printf("[CTRL] RESET pulse %lu ms%s\n", static_cast<unsigned long>(durationMs),
                      gResetLatched ? " (latched hold will keep line LOW)" : "");
    }
}

void ServicePulseTimers() {
    const uint32_t now = millis();
    if (gRunPulseActive && static_cast<int32_t>(now - gRunPulseReleaseMs) >= 0) {
        gRunPulseActive = false;
        UpdateRunOutput();
        if (!gRunLatched) {
            Serial.println("[CTRL] RUN pulse complete");
        }
    }
    if (gResetPulseActive && static_cast<int32_t>(now - gResetPulseReleaseMs) >= 0) {
        gResetPulseActive = false;
        UpdateResetOutput();
        if (!gResetLatched) {
            Serial.println("[CTRL] RESET pulse complete");
        }
    }
}

void ProcessHostCommand(const String &rawLine) {
    String line = rawLine;
    line.trim();
    if (!line.length()) {
        return;
    }

    String upper = line;
    upper.toUpperCase();

    if (upper == "STATUS" || upper == "RUN?" || upper == "RST?" || upper == "RESET?" || upper == "HELP") {
        Serial.println("[CTRL] Commands: RUN=0|1, RST=0|1, PWR=0|1, PULSE RUN=<ms>, PULSE RST=<ms>, STATUS");
        EmitControlStatus();
        return;
    }

    if (upper.startsWith("RUN=")) {
        bool active = false;
        if (ParseActiveToken(line.substring(4), active)) {
            HandleLatchedCommand(true, active);
        } else {
            Serial.println("[CTRL] RUN expects 0/1 or ON/OFF");
        }
        return;
    }

    if (upper.startsWith("RST=") || upper.startsWith("RESET=")) {
        const int offset = upper.startsWith("RST=") ? 4 : 6;
        bool active = false;
        if (ParseActiveToken(line.substring(offset), active)) {
            HandleLatchedCommand(false, active);
        } else {
            Serial.println("[CTRL] RESET expects 0/1 or ON/OFF");
        }
        return;
    }

    if (upper.startsWith("PWR=") || upper.startsWith("POWER=")) {
        const int offset = upper.startsWith("PWR=") ? 4 : 6;
        bool active = false;
        if (ParseActiveToken(line.substring(offset), active)) {
            HandlePowerCommand(active);
        } else {
            Serial.println("[CTRL] POWER expects 0/1 or ON/OFF");
        }
        return;
    }

    if (upper.startsWith("PULSE")) {
        const int spacePos = upper.indexOf(' ');
        if (spacePos < 0) {
            Serial.println("[CTRL] Usage: PULSE RUN=<ms>");
            return;
        }
        String targetUpper = upper.substring(spacePos + 1);
        String targetRaw = line.substring(spacePos + 1);
        targetUpper.trim();
        targetRaw.trim();

        bool isRun = false;
        int valuePos = -1;
        if (targetUpper.startsWith("RUN=")) {
            isRun = true;
            valuePos = 4;
        } else if (targetUpper.startsWith("RST=")) {
            isRun = false;
            valuePos = 4;
        } else if (targetUpper.startsWith("RESET=")) {
            isRun = false;
            valuePos = 6;
        } else {
            Serial.println("[CTRL] PULSE expects RUN= or RST=");
            return;
        }

        String durationToken = targetRaw.substring(valuePos);
        durationToken.trim();
        long duration = durationToken.toInt();
        if (duration <= 0) {
            Serial.println("[CTRL] Pulse duration must be >0 ms");
            return;
        }

        HandlePulseCommand(isRun, static_cast<uint32_t>(duration));
        return;
    }

    Serial.println("[CTRL] Unknown command – type HELP");
}

void ServiceHostCommands() {
    while (Serial && Serial.available() > 0) {
        const char ch = static_cast<char>(Serial.read());
        if (ch == '\r') {
            continue;
        }
        if (ch == '\n') {
            ProcessHostCommand(gUsbLine);
            gUsbLine = "";
        } else if (ch == '\b' || ch == 0x7F) {
            if (gUsbLine.length() > 0) {
                gUsbLine.remove(gUsbLine.length() - 1);
            }
        } else if (gUsbLine.length() < kUsbCommandMaxLen) {
            gUsbLine += ch;
        }
    }
}

bool HexWindowActive() {
    if (!kEnableHexBootWindow) {
        return false;
    }
    const uint32_t elapsed = millis() - gBootStartMs;
    return elapsed < kHexBootWindowMs;
}

void EmitPrefix(const ChannelState &channel, uint32_t stampMs) {
    const uint32_t deltaMs = (channel.lastStampMs == 0) ? 0 : (stampMs - channel.lastStampMs);
    Serial.print('[');
    Serial.print(F("ms="));
    Serial.printf("%010lu", static_cast<unsigned long>(stampMs));
    Serial.print(F(" Δ="));
    Serial.printf("%04lu", static_cast<unsigned long>(deltaMs));
    Serial.print(' ');
    Serial.print(channel.tag);
    Serial.print(F("] "));
}

void EmitHexPayload(const uint8_t *data, size_t length) {
    for (size_t i = 0; i < length; ++i) {
        if (i > 0) {
            Serial.print(' ');
        }
        const uint8_t value = data[i];
        Serial.print(kHexDigits[value >> 4]);
        Serial.print(kHexDigits[value & 0x0F]);
    }
}

void EmitTextPayload(const uint8_t *data, size_t length) {
    for (size_t i = 0; i < length; ++i) {
        const uint8_t value = data[i];
        if (kForwardRawBytes || value == '\r') {
            Serial.write(value);
        } else if (IsPrintableAscii(value)) {
            Serial.write(value);
        } else {
            Serial.write('?');
        }
    }
}

void CheckFirstLineWarning(ChannelState &channel, const uint8_t *data, size_t length) {
    if (!channel.firstLinePending || channel.warnedMismatch) {
        return;
    }
    size_t printable = 0;
    size_t noise = 0;
    for (size_t i = 0; i < length; ++i) {
        const uint8_t value = data[i];
        if (value == '\r' || value == '\n') {
            continue;
        }
        if (IsPrintableAscii(value)) {
            ++printable;
        } else {
            ++noise;
        }
    }
    if (noise >= kNonPrintableWarningThreshold && noise >= printable) {
        Serial.print(F("[WARN] Possible "));
        Serial.print(channel.tag);
        Serial.println(F(" baud mismatch; first line contained non-printable bytes"));
        channel.warnedMismatch = true;
    }
    channel.firstLinePending = false;
}

void FlushChannelBuffer(ChannelState &channel, bool overflow) {
    if (channel.length == 0) {
        return;
    }

    const bool hexMode = HexWindowActive();
    const bool endsWithLf = channel.buffer[channel.length - 1] == '\n';

    size_t payloadLength = channel.length;
    if (!hexMode && endsWithLf && payloadLength > 0) {
        --payloadLength;
    }
    if (!hexMode && !kForwardRawBytes && payloadLength > 0 && channel.buffer[payloadLength - 1] == '\r') {
        --payloadLength;
    }

    const uint32_t stampMs = millis() - gBootStartMs;
    EmitPrefix(channel, stampMs);
    if (hexMode) {
        EmitHexPayload(channel.buffer, channel.length);
        if (overflow) {
            Serial.print(F(" …"));
        }
        Serial.println();
    } else {
        EmitTextPayload(channel.buffer, payloadLength);
        if (overflow) {
            Serial.println(F(" …"));
        } else {
            Serial.println();
            CheckFirstLineWarning(channel, channel.buffer, payloadLength);
        }
    }

    channel.length = 0;
    channel.lastStampMs = stampMs;
}

void DrainUntilNewline(ChannelState &channel) {
    const uint32_t deadline = millis() + kSyncTimeoutMs;
    bool newlineSeen = false;
    while ((int32_t)(deadline - millis()) > 0) {
        while (channel.serial.available() > 0) {
            const int value = channel.serial.read();
            if (value < 0) {
                continue;
            }
            if (value == '\n') {
                newlineSeen = true;
                break;
            }
        }
        if (newlineSeen) {
            break;
        }
        yield();
    }

    while (channel.serial.available() > 0) {
        channel.serial.read();
    }

    channel.syncedNewline = newlineSeen;
}

void ServiceChannel(ChannelState &channel) {
    while (channel.serial.available() > 0) {
        const int value = channel.serial.read();
        if (value < 0) {
            break;
        }
        gLastActivityMs = millis();
        if (channel.length >= kLineBufferSize - 1) {
            FlushChannelBuffer(channel, true);
        }
        channel.buffer[channel.length++] = static_cast<uint8_t>(value);
        if (static_cast<uint8_t>(value) == '\n') {
            FlushChannelBuffer(channel, false);
        }
    }
}
}  // namespace

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    pinMode(kCcTxRxPin, INPUT);   // pin 7
    pinMode(kXpbTxRxPin, INPUT);  // pin 0
    digitalWrite(kRunCtrlPin, HIGH);
    digitalWrite(kResetCtrlPin, HIGH);
    pinMode(kRunCtrlPin, OUTPUT_OPENDRAIN);
    pinMode(kResetCtrlPin, OUTPUT_OPENDRAIN);
    pinMode(kPowerCtrlPin, OUTPUT);
    gPowerAsserted = false;
    UpdatePowerOutput();
    UpdateRunOutput();
    UpdateResetOutput();

    Serial.begin(kUsbBaud);
    while (!Serial) {
        // Standby: slow blink until host opens USB serial
        gLedIntervalMs = kStandbyBlinkMs;
        UpdateLedBlink();
        delay(10);
    }

    Serial.println("[INFO] RUN/RESET/POWER control ready (type HELP)");
    EmitControlStatus();

    Serial1.begin(kTtlBaud);
    Serial2.begin(kTtlBaud);

    DrainUntilNewline(gCcChannel);
    DrainUntilNewline(gXpbChannel);

    gBootStartMs = millis();
    gLastActivityMs = gBootStartMs;

    // Active: faster blink while sniffing
    gLedIntervalMs = kActiveBlinkMs;

    Serial.print(F("[INFO] CC_TX sniff baud="));
    Serial.print(kTtlBaud);
    Serial.print(F(", XPB_TX sniff baud="));
    Serial.print(kTtlBaud);
    Serial.print(F(", USB="));
    Serial.println(kUsbBaud);

    if (kEnableHexBootWindow) {
        Serial.print(F("[INFO] Hex diagnostic window active for "));
        Serial.print(kHexBootWindowMs);
        Serial.println(F(" ms"));
    }

    if (!gCcChannel.syncedNewline) {
        Serial.println(F("[WARN] CC_TX channel sync timed out; first line may be partial"));
    }
    if (!gXpbChannel.syncedNewline) {
        Serial.println(F("[WARN] XPB_TX channel sync timed out; first line may be partial"));
    }
}

void loop() {
    ServiceHostCommands();
    ServicePulseTimers();
    ServiceChannel(gCcChannel);
    ServiceChannel(gXpbChannel);
    // Policy: if USB not connected or idle beyond timeout -> slow blink; else fast
    const uint32_t now = millis();
    if (!Serial || (now - gLastActivityMs) > kIdleTimeoutMs) {
        gLedIntervalMs = kStandbyBlinkMs;
    } else {
        gLedIntervalMs = kActiveBlinkMs;
    }
    UpdateLedBlink();
}
