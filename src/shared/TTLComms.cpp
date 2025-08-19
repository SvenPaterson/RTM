// TTLComms.cpp
#include "TTLComms.h"

#define XPB_TX_ECHO_USB 1

// 1) Command sender: always carries REF and expects an ACK/response
void TTLComms::sendCommand(const char* base, MessageType type) {
    // Force an ACK policy if caller passed INFO/NORMAL
    if (type == MessageType::INFO || type == MessageType::NORMAL) {
        type = MessageType::IMPORTANT;
    }

    // Per-type policy
    uint8_t  maxRetries = 2;
    uint16_t toMs       = 200;
    switch (type) {
      case MessageType::CRITICAL:  maxRetries = 3; toMs = 100; break;
      case MessageType::IMPORTANT: maxRetries = 2; toMs = 300; break;
      default:                     maxRetries = 2; toMs = 200; break;
    }

    // Build payload with ;REF= (preserve if caller already attached one)
    uint16_t ref = nextRef_++;
    char payload[MAX_MSG_LEN + 1];
    const char *pRef = strstr(base, ";REF=");
    if (pRef) {
        // Track the caller-supplied REF so we correlate properly
        ref = (uint16_t)atoi(pRef + 5);
        snprintf(payload, sizeof(payload), "%s", base);
    } else {
        snprintf(payload, sizeof(payload), "%s;REF=%u", base, (unsigned)ref);
    }

    // Encode with checksum + LF
    char msg[80];
    const uint8_t checksum = this->calculateXOR(payload);
    snprintf(msg, sizeof(msg), "%s:%02X\n", payload, checksum);

    // TX echo
    if (logRx_) {
        char line[96];
        if (rxTag_) snprintf(line, sizeof(line), "[TX %s] %s", rxTag_, msg);
        else        snprintf(line, sizeof(line), "[TX] %s", msg);
        usbLog(line);
    }

    // Send
    serialSend(msg);

    // Arm pending (ACK expected)
    pendingMsg_    = { String(msg), millis(), 0, maxRetries, true, ref };
    waitingForAck_ = true;
    ackTimeoutMs_  = toMs;
    pendingRef_    = ref;
}

// 2) Typed sender: INFO/NORMAL (no ACK), IMPORTANT/CRITICAL (ACK + REF)
void TTLComms::sendMessage(const char* data, MessageType type) {
    const bool wantsAck =
        (type == MessageType::CRITICAL || type == MessageType::IMPORTANT);

    // Per-type policy
    uint8_t  maxRetries = 0;
    uint16_t toMs       = ACK_TIMEOUT_MS;
    switch (type) {
      case MessageType::CRITICAL:  maxRetries = 3; toMs = 100; break;
      case MessageType::IMPORTANT: maxRetries = 2; toMs = 300; break;
      case MessageType::NORMAL:    maxRetries = 0; toMs = 0;   break;
      case MessageType::INFO:      maxRetries = 0; toMs = 0;   break;
    }

    // Prepare payload (append ;REF= only for ACK-carrying classes)
    char payload[MAX_MSG_LEN + 1];
    uint16_t usedRef = 0;

    if (!wantsAck) {
        snprintf(payload, sizeof(payload), "%s", data);
    } else {
        const char *pRef = strstr(data, ";REF=");
        if (pRef) {
            usedRef = (uint16_t)atoi(pRef + 5);
            snprintf(payload, sizeof(payload), "%s", data);
        } else {
            usedRef = nextRef_++;
            snprintf(payload, sizeof(payload), "%s;REF=%u", data, (unsigned)usedRef);
        }
    }

    // Encode with checksum + LF
    char msg[80];
    const uint8_t checksum = this->calculateXOR(payload);
    snprintf(msg, sizeof(msg), "%s:%02X\n", payload, checksum);

    // TX echo
    if (logRx_) {
        char line[96];
        if (rxTag_) snprintf(line, sizeof(line), "[TX %s] %s", rxTag_, msg);
        else        snprintf(line, sizeof(line), "[TX] %s", msg);
        usbLog(line);
    }

    // Send
    serialSend(msg);

    // Only arm pending for ACK classes
    if (wantsAck) {
        pendingMsg_    = { String(msg), millis(), 0, maxRetries, true, usedRef };
        waitingForAck_ = true;
        ackTimeoutMs_  = toMs;
        pendingRef_    = usedRef;
    }
}

// 3) Legacy bool overload → map onto typed policy cleanly
void TTLComms::sendMessage(const char* data, bool needsAck) {
    if (needsAck) sendMessage(data, MessageType::IMPORTANT);
    else          sendMessage(data, MessageType::INFO);
}

void TTLComms::checkRetries() {
    if (!waitingForAck_) return;

    if (millis() - pendingMsg_.sentTime > ackTimeoutMs_) {
        if (pendingMsg_.retryCount < pendingMsg_.maxRetries) {
            pendingMsg_.retryCount++;
            pendingMsg_.sentTime = millis();
            serialSend(pendingMsg_.encoded.c_str());
        }
        else {
            waitingForAck_ = false;
        }
    }
}

void TTLComms::checkForMessages() {
    while (serialAvailable()) {
        char c = serialRead();
        if (c == '\r') continue;
        if (c == '\n') {
            if (validateMessage(incomingMsg_)) {
                processMessage(incomingMsg_);
            } else {
                onBadChecksum(incomingMsg_);
            }
            incomingMsg_ = "";
        } else if (incomingMsg_.length() < MAX_MSG_LEN) {
            incomingMsg_ += c;
        }
    }
}

uint8_t TTLComms::calculateXOR(const char* data) {
    // for error checking a message
    uint8_t checksum = 0;
    while (*data) {
        checksum ^= *data++;
    }
    return checksum;
}

bool TTLComms::validateMessage(const String& msg) {
    const int delim = msg.lastIndexOf(':');
    const size_t len = msg.length();
    if (delim < 0) return false;

    // Expect exactly 2 hex digits after ':'
    const size_t csLen = len - (size_t)delim - 1;
    if (csLen != 2) return false;

    const char c0 = msg[delim + 1];
    const char c1 = msg[delim + 2];
    auto isHex = [](char c) {
        return (c >= '0' && c <= '9') ||
               (c >= 'A' && c <= 'F') ||
               (c >= 'a' && c <= 'f');
    };
    if (!isHex(c0) || !isHex(c1)) return false;

    auto hexVal = [](char c) -> uint8_t {
        if (c >= '0' && c <= '9') return uint8_t(c - '0');
        if (c >= 'A' && c <= 'F') return uint8_t(c - 'A' + 10);
        return uint8_t(c - 'a' + 10);
    };
    const uint8_t given = (hexVal(c0) << 4) | hexVal(c1);

    // XOR of everything before ':'
    uint8_t calc = 0;
    for (int i = 0; i < delim; ++i) calc ^= (uint8_t)msg[i];

    if (calc != given) {
        // Optional: one-shot hex dump (guarded) – pure C strings for usbLog()
        #ifndef TTL_DUMP_BAD_DISABLE
        static bool dumpedBadOnce = false;
        if (!dumpedBadOnce && logRx_) {
            dumpedBadOnce = true;
            char hex[256]; size_t pos = 0;
            for (size_t i = 0; i < len && pos + 3 < sizeof(hex); ++i)
                pos += snprintf(hex + pos, sizeof(hex) - pos, "%02X ", (uint8_t)msg[i]);

            if (rxTag_) {
                char line[320];
                snprintf(line, sizeof(line), "[RX BAD %s] len=%u HEX=%s",
                         rxTag_, (unsigned)len, hex);
                usbLog(line);
            } else {
                char line[320];
                snprintf(line, sizeof(line), "[RX BAD] len=%u HEX=%s",
                         (unsigned)len, hex);
                usbLog(line);
            }
        }
        #endif
        return false;
    }

    return true;
}

void TTLComms::processMessage(const String& msg) {
    int lastColon = msg.lastIndexOf(':');
    String data = (lastColon >= 0) ? msg.substring(0, lastColon) : msg;

    // handle ACKs centralling so retries stop
    if (data.startsWith("ACK:")) {
        waitingForAck_ = false;
        return;
    }

    // Optional RX log on the receiver’s USB
    if (logRx_) {
        char line[96];
        if (rxTag_) snprintf(line, sizeof(line), "[RX %s] %s", rxTag_, data.c_str());
        else        snprintf(line, sizeof(line), "[RX] %s", data.c_str());
        usbLog(line);
    }

    // --- Correlate replies to pending command (explicit ACK or data reply) ---
    if (waitingForAck_) {
        // 1) Explicit ACK;…;REF=n
        if (msg.startsWith("ACK;")) {
            int pos = msg.indexOf(F(";REF="));
            if (pos > 0) {
                uint16_t r = (uint16_t)msg.substring(pos + 5).toInt();
                if (r == pendingRef_) {
                    waitingForAck_ = false;
                    pendingRef_ = 0;
                }
            } else {
                // Legacy ACK without REF: accept for backward compatibility
                waitingForAck_ = false;
                pendingRef_ = 0;
            }
            // Do not early-return; let app see the ACK if it wants
        } else {
            // 2) Data reply that mirrors ;REF=n (e.g., SW;…;REF=n)
            int pos = msg.indexOf(F(";REF="));
            if (pos > 0) {
                uint16_t r = (uint16_t)msg.substring(pos + 5).toInt();
                if (r == pendingRef_) {
                    waitingForAck_ = false;
                    pendingRef_ = 0;
                }
            }
        }
    }

    onMessageReceived(data);
}

String TTLComms::kvGet(const String &frame, const char *key) {
    int k = frame.indexOf(key);
    if (k < 0) return String();
    k += (int)strlen(key);
    int e = frame.indexOf(';', k);
    if (e < 0) e = frame.length();
    return frame.substring(k, e);
}

int TTLComms::kvGetIntClamped(const String &frame, const char *key,
                                int defVal, int minV, int maxV) {
    String s = kvGet(frame, key);
    if (!s.length()) return defVal;
    long v = s.toInt();
    if (v < minV) v = minV;
    if (v > maxV) v = maxV;
    return (int)v;
}

double TTLComms::kvGetDouble(const String &frame, const char *key, double defVal) {
        String s = kvGet(frame, key);
        return s.length() ? s.toFloat() : defVal;
    }

void TTLComms::setRxUsbLogging(bool enabled, const char *peerTag) {
    logRx_ = enabled;
    rxTag_ = peerTag;
}