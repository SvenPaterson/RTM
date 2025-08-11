// TTLComms.cpp
#include "TTLComms.h"

void TTLComms::sendMessage(const char* data) {
    char msg[80];
    uint8_t checksum = calculateXOR(data);
    snprintf(msg, sizeof(msg), "%s:%02X\n", data, checksum);
    serialSend(msg);
}

void TTLComms::checkForMessages() {
    while (serialAvailable()) {
        char c = serialRead();
        
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
    String data = msg.substring(0, lastColon);
    onMessageReceived(data);
}