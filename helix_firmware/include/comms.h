// - Use a clear, checksum-verified packet format
// - Teensy → Pi: Odometry, IMU, sensor status
// - Pi → Teensy: Velocity commands, LED mode, etc.

// Comms.h
// UART communication helper for Teensy <-> Raspberry Pi
// Designed for Teensy 4.1 (Arduino-style environment).
// Author: ChatGPT (adapted to Helix project)
// Date: 2025-10-14

#ifndef COMMS_H
#define COMMS_H

#include <Arduino.h>
#include <functional>
#include <vector>
#include <stdint.h>

// Public API for comms module
class Comms {
public:
    // Message callback signature: (msg_id, payload_ptr, payload_len)
    using MsgCallback = std::function<void(uint8_t, const uint8_t*, uint16_t)>;

    // Constructor
    // serialPort: reference to the Stream/HardwareSerial (e.g., Serial1)
    // baud: UART baud rate (e.g., 115200, 500000)
    Comms(Stream &serialPort, uint32_t baud = 115200U);

    // Initialize hardware serial and internal state - call from setup()
    void begin();

    // Call frequently from loop() (non-blocking) to process RX and TX
    void poll();

    // Register a callback for received messages
    void onMessage(MsgCallback cb);

    // Send a message with a single byte message id and payload pointer/length.
    // Non-blocking: returns true if queued, false if queue full / too large.
    bool send(uint8_t msg_id, const uint8_t* payload, uint16_t payload_len);

    // Convenience send for std::vector
    bool send(uint8_t msg_id, const std::vector<uint8_t>& payload);

    // Set read timeout (ms) used by RX state machine to detect stalled frames
    void setRxTimeout(uint32_t ms);

    // Destructor
    ~Comms();

    // Disable copy
    Comms(const Comms&) = delete;
    Comms& operator=(const Comms&) = delete;

    // Static constants
    static const uint8_t SYNC0 = 0xAA;
    static const uint8_t SYNC1 = 0x55;
    static const uint16_t MAX_PAYLOAD = 1024; // tuneable limit

private:
    Stream &serial;
    uint32_t baudrate;

    // RX state machine
    enum RxState {
        RX_SYNC0,
        RX_SYNC1,
        RX_LEN_LO,
        RX_LEN_HI,
        RX_MSGID,
        RX_PAYLOAD,
        RX_CRC_LO,
        RX_CRC_HI
    } rxState;

    uint16_t rxLen;
    uint16_t rxIndex;
    uint8_t rxMsgId;
    uint8_t rxBuffer[MAX_PAYLOAD];
    uint16_t rxCrc;

    uint32_t lastRxByteTime; // millis of last byte, for timeout
    uint32_t rxTimeoutMs;

    // TX queue (simple circular buffer of bytes)
    static const size_t TX_QUEUE_SIZE = 4096; // total bytes queued
    uint8_t *txQueue;
    size_t txHead;
    size_t txTail;
    size_t txCount;

    // outgoing packet build helpers
    void queueBytes(const uint8_t* data, size_t len);
    bool canQueue(size_t len) const;

    // low-level send (push packet to queue)
    bool enqueuePacket(uint8_t msg_id, const uint8_t* payload, uint16_t payload_len);

    // CRC helper
    static uint16_t crc16_ccitt(const uint8_t* data, size_t len, uint16_t init = 0xFFFF);

    // internal callback
    MsgCallback messageCb;

    // helper to flush tx queue to serial (non-blocking)
    void flushTx();

    // called when a full message received
    void handleCompletePacket();

    // time helper
    uint32_t nowMs() const { return millis(); }
};

#endif // COMMS_H
