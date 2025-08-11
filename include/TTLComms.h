// TTLComms.h - Shared TTL communication class
#pragma once

#include <Arduino.h>

class TTLComms {
public:
    // Abstract interface for different serial implementations
    virtual void serialSend(const char* data) = 0;
    virtual bool serialAvailable() = 0;
    virtual char serialRead() = 0;
    virtual int serialPeek() = 0;
    
    // Common functionality
    void sendMessage(const char* data);
    void checkForMessages();
    
    // Message callback - override in derived classes
    // probably not needed, or move to universal TTLComms definition
    virtual void onMessageReceived(const String& data) = 0;
    virtual void onBadChecksum(const String& rawMsg) = 0;
    
protected:
    // Common message processing
    uint8_t calculateXOR(const char* data);
    bool validateMessage(const String& msg);
    void processMessage(const String& msg);
    
private:
    String incomingMsg_ = "";
    static constexpr size_t MAX_MSG_LEN = 79;
};