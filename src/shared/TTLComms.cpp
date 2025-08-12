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
    if (lastColon == -1 || lastColon >= msg.length() - 1) {
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

    onMessageReceived(data);
}