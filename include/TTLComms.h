// TTLComms.h - Shared TTL communication class
#pragma once

#include <Arduino.h>

struct PendingMessage {
    String      encoded; // full "DATA:CHK\n"
    uint32_t    sentTime;
    uint8_t     retryCount;
    uint8_t     maxRetries;
    bool        needsAck;
    uint16_t    ref;
};

enum class MessageType {
    CRITICAL,       // State-change, safety: ACK + REF, 3 retries, 100ms timeout
    IMPORTANT,      // State-change, idempotent: ACK + REF, 2 retries, 200ms timeout
    NORMAL,         // Notifications / request-response: no ACK (response is the ACK)
    INFO            // Telemetry/heartbeat: no ACK
};

class TTLComms {
public:
    // Abstract interface for different serial implementations
    virtual void serialSend(const char* data) = 0;
    virtual bool serialAvailable() = 0;
    virtual char serialRead() = 0;
    virtual int  serialPeek() = 0;
    
    // Common functionality
    void beginBase() { incomingMsg_.reserve(MAX_MSG_LEN + 8); } // preallocate memory
    void sendMessage(const char* data, bool needsAck = false);
    void sendMessage(const char* data, MessageType type);
    // Convenience for commands that should carry a REF and expect an ACK/response
    void sendCommand(const char* base, MessageType type = MessageType::IMPORTANT);
    void checkRetries();
    void checkForMessages();
    void setRxUsbLogging(bool enabled, const char *peerTag = nullptr);

    // Message callback - override in derived classes
    // probably not needed, or move to universal TTLComms definition
    virtual void onMessageReceived(const String& data) = 0;
    virtual void onBadChecksum(const String& rawMsg) = 0;

    // TEST ONLY: inject decoded frame straight to handler
    void testInject(const String &frame) { onMessageReceived(frame); }
    
protected:
    // Common message processing
    uint8_t calculateXOR(const char* data);
    bool validateMessage(const String& msg);
    void processMessage(const String& msg);
    String kvGet(const String &frame, const char *key);
    int kvGetIntClamped(const String &frame, const char *key,
                        int defVal, int minV, int maxV);
    double kvGetDouble(const String &frame, const char *key, double defVal);
    virtual void usbLog(const char *s) { /* default: no-op */}
    
private:
    static constexpr uint32_t ACK_TIMEOUT_MS = 500;
    static constexpr uint8_t MAX_RETRIES = 5;

    bool trySplitGluedFrames_(const String &line);
    
    PendingMessage pendingMsg_;
    bool waitingForAck_ = false;
    uint16_t ackTimeoutMs_ = ACK_TIMEOUT_MS;
    uint16_t nextRef_    = 1;   // rolling correlation token
    uint16_t pendingRef_ = 0;   // REF of the in-flight cmd (if any)

    static constexpr size_t MAX_MSG_LEN = 160;
    String incomingMsg_ = ""; 

    bool        logRx_ = false;
    const char *rxTag_ = nullptr;
};