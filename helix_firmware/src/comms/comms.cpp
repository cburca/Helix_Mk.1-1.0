// Comms.cpp
#include "comms.h"

// Constructor
Comms::Comms(Stream &serialPort, uint32_t baud)
    : serial(serialPort), baudrate(baud),
      rxState(RX_SYNC0),
      rxLen(0), rxIndex(0), rxMsgId(0), rxCrc(0),
      lastRxByteTime(0), rxTimeoutMs(100), // default 100 ms timeout
      txHead(0), txTail(0), txCount(0), messageCb(nullptr)
{
    txQueue = (uint8_t*)malloc(TX_QUEUE_SIZE);
    // if malloc fails, txQueue will be null; send() will fail gracefully
}

Comms::~Comms() {
    if (txQueue) free(txQueue);
}

void Comms::begin() {
    // If serial is a HardwareSerial, begin() configures UART speed.
    // If using Serial1 on Teensy: Serial1.begin(baudrate);
    // We check for HardwareSerial via dynamic_cast; Stream doesn't have begin() itself.
    HardwareSerial* hs = dynamic_cast<HardwareSerial*>(&serial);
    if (hs) {
        hs->begin(baudrate);
    } else {
        // If not HardwareSerial, assume already opened (e.g., SoftSerial wrapper).
    }

    // reset RX state machine
    rxState = RX_SYNC0;
    rxLen = 0;
    rxIndex = 0;
    lastRxByteTime = nowMs();
}

void Comms::setRxTimeout(uint32_t ms) {
    rxTimeoutMs = ms;
}

void Comms::onMessage(MsgCallback cb) {
    messageCb = cb;
}

bool Comms::send(uint8_t msg_id, const uint8_t* payload, uint16_t payload_len) {
    if (!payload && payload_len > 0) return false;
    if (payload_len > MAX_PAYLOAD) return false;

    // Packet = SYNC0 SYNC1 LEN(2 bytes little-endian) MSGID payload CRC16(2 bytes little-endian)
    // Total needed bytes:
    size_t total = 2 + 2 + 1 + payload_len + 2;
    if (!canQueue(total)) return false;

    return enqueuePacket(msg_id, payload, payload_len);
}

bool Comms::send(uint8_t msg_id, const std::vector<uint8_t>& payload) {
    return send(msg_id, payload.data(), (uint16_t)payload.size());
}

bool Comms::canQueue(size_t len) const {
    if (!txQueue) return false;
    return (txCount + len) <= TX_QUEUE_SIZE;
}

void Comms::queueBytes(const uint8_t* data, size_t len) {
    if (!txQueue) return;
    for (size_t i=0;i<len;i++) {
        txQueue[txHead] = data[i];
        txHead = (txHead + 1) % TX_QUEUE_SIZE;
    }
    txCount += len;
}

bool Comms::enqueuePacket(uint8_t msg_id, const uint8_t* payload, uint16_t payload_len) {
    if (!txQueue) return false;

    // header
    uint8_t header[2] = { SYNC0, SYNC1 };
    queueBytes(header, 2);

    // length (little endian)
    uint8_t lenBytes[2];
    lenBytes[0] = payload_len & 0xFF;
    lenBytes[1] = (payload_len >> 8) & 0xFF;
    queueBytes(lenBytes, 2);

    // message id
    queueBytes(&msg_id, 1);

    // payload
    if (payload_len) queueBytes(payload, payload_len);

    // compute CRC over (LEN_LO LEN_HI MSGID payload...)
    // Build temporary buffer to compute CRC
    size_t crcInputLen = 2 + 1 + payload_len;
    uint8_t* crcBuf = (uint8_t*)malloc(crcInputLen);
    if (!crcBuf) return false; // out of memory - shouldn't happen on small embedded
    crcBuf[0] = lenBytes[0];
    crcBuf[1] = lenBytes[1];
    crcBuf[2] = msg_id;
    if (payload_len) memcpy(crcBuf + 3, payload, payload_len);
    uint16_t crc = crc16_ccitt(crcBuf, crcInputLen);
    free(crcBuf);

    uint8_t crcBytes[2];
    crcBytes[0] = crc & 0xFF;
    crcBytes[1] = (crc >> 8) & 0xFF;
    queueBytes(crcBytes, 2);

    // Try flushing now (best-effort non-blocking)
    flushTx();

    return true;
}

void Comms::flushTx() {
    // Write as many queued bytes as hardware allows without blocking
    // Stream::write in Arduino is often synchronous; HardwareSerial::write will push to TX buffer.
    // We'll write in chunks to avoid huge blocking.
    if (!txQueue) return;
    HardwareSerial* hs = dynamic_cast<HardwareSerial*>(&serial);
    if (!hs) {
        // Fallback: write byte-by-byte
        while (txCount > 0 && serial.availableForWrite()) {
            uint8_t b = txQueue[txTail];
            serial.write(b);
            txTail = (txTail + 1) % TX_QUEUE_SIZE;
            txCount--;
        }
        return;
    }

    // For HardwareSerial, use write with contiguous block optimization
    while (txCount > 0 && hs->availableForWrite()) {
        // Determine contiguous chunk size
        size_t chunk = (txTail < txHead) ? (txHead - txTail) : (TX_QUEUE_SIZE - txTail);
        if (chunk > txCount) chunk = txCount;
        // Limit chunk to availableForWrite
        size_t canWrite = (size_t)hs->availableForWrite();
        if (canWrite == 0) break;
        if (chunk > canWrite) chunk = canWrite;

        // write chunk
        hs->write(txQueue + txTail, chunk);
        txTail = (txTail + chunk) % TX_QUEUE_SIZE;
        txCount -= chunk;
    }
}

// poll(): call from loop()
void Comms::poll() {
    // RX processing: read all available bytes
    while (serial.available()) {
        int c = serial.read();
        if (c < 0) break;
        uint8_t b = (uint8_t)c;
        lastRxByteTime = nowMs();

        switch (rxState) {
            case RX_SYNC0:
                if (b == SYNC0) rxState = RX_SYNC1;
                break;
            case RX_SYNC1:
                if (b == SYNC1) rxState = RX_LEN_LO;
                else rxState = (b == SYNC0) ? RX_SYNC1 : RX_SYNC0;
                break;
            case RX_LEN_LO:
                rxLen = (uint16_t)b;
                rxState = RX_LEN_HI;
                break;
            case RX_LEN_HI:
                rxLen |= ((uint16_t)b << 8);
                if (rxLen > MAX_PAYLOAD) {
                    // invalid length -> reset state machine
                    rxState = RX_SYNC0;
                    rxLen = 0;
                } else {
                    rxIndex = 0;
                    rxState = RX_MSGID;
                }
                break;
            case RX_MSGID:
                rxMsgId = b;
                if (rxLen == 0) rxState = RX_CRC_LO;
                else rxState = RX_PAYLOAD;
                break;
            case RX_PAYLOAD:
                rxBuffer[rxIndex++] = b;
                if (rxIndex >= rxLen) rxState = RX_CRC_LO;
                break;
            case RX_CRC_LO:
                rxCrc = b;
                rxState = RX_CRC_HI;
                break;
            case RX_CRC_HI:
                rxCrc |= ((uint16_t)b << 8);
                // Validate CRC: compute over LEN_LO LEN_HI MSGID payload...
                {
                    size_t inputLen = 2 + 1 + rxLen;
                    uint8_t* tmp = (uint8_t*)malloc(inputLen);
                    if (!tmp) {
                        // OOM - just reset
                        rxState = RX_SYNC0;
                        break;
                    }
                    tmp[0] = (uint8_t)(rxLen & 0xFF);
                    tmp[1] = (uint8_t)((rxLen >> 8) & 0xFF);
                    tmp[2] = rxMsgId;
                    if (rxLen) memcpy(tmp + 3, rxBuffer, rxLen);
                    uint16_t calc = crc16_ccitt(tmp, inputLen);
                    free(tmp);
                    if (calc == rxCrc) {
                        // good packet
                        handleCompletePacket();
                    } else {
                        // CRC mismatch - ignore
                    }
                    rxState = RX_SYNC0;
                }
                break;
        }
    }

    // Simple RX timeout handling: if waiting for rest of frame and timeout exceeded, reset
    if (rxState != RX_SYNC0 && (nowMs() - lastRxByteTime) > rxTimeoutMs) {
        rxState = RX_SYNC0;
    }

    // Flush TX queue to hardware
    flushTx();
}

void Comms::handleCompletePacket() {
    if (messageCb) {
        messageCb(rxMsgId, rxBuffer, rxLen);
    }
}

// CRC16-CCITT (XModem style poly 0x1021) implementation
uint16_t Comms::crc16_ccitt(const uint8_t* data, size_t len, uint16_t init) {
    uint16_t crc = init;
    for (size_t i=0;i<len;i++) {
        crc ^= ((uint16_t)data[i]) << 8;
        for (uint8_t j=0;j<8;j++) {
            if (crc & 0x8000) crc = (uint16_t)((crc << 1) ^ 0x1021);
            else crc <<= 1;
        }
    }
    return crc;
}
