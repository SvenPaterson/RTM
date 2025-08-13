// TTLComms.cpp
#include "TTLComms.h"

void TTLComms::sendMessage(const char* data, bool needsAck) {
    char msg[80];
    uint8_t checksum = calculateXOR(data);
    snprintf(msg, sizeof(msg), "%s:%02X\r\n", data, checksum); // CRLF

    serialSend(msg);

    if (needsAck) {
        pendingMsg_ = { String(msg), millis(), 0, MAX_RETRIES, true };
        waitingForAck_ = true;
    }
}

void TTLComms::sendMessage(const char* data, MessageType type) {
    bool needsAck = (type != MessageType::INFO);
    sendMessage(data, needsAck);
}

void TTLComms::checkRetries() {
    if (!waitingForAck_) return;

    if (millis() - pendingMsg_.sentTime > ACK_TIMEOUT_MS) {
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
    // read last byte and perform error check
    int lastColon = msg.lastIndexOf(':');
    size_t len = msg.length();
    if (lastColon == -1 || lastColon >= len - 1) {
        return false;
    }
    
    String data = msg.substring(0, lastColon);
    String checksumStr = msg.substring(lastColon + 1);
    
    uint8_t expectedChecksum = calculateXOR(data.c_str());
    uint8_t receivedChecksum = (uint8_t)strtol(checksumStr.c_str(), nullptr, 16);
    
    return (expectedChecksum == receivedChecksum);
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