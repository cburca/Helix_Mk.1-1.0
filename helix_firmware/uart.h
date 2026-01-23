#ifndef UART_H
#define UART_H

#include <Arduino.h>

// [SYNC][TYPE][LEN][PAYLOAD][CRC]
#define UART_SYNC_1  0xAA
#define UART_SYNC_2  0x55

enum PacketType : uint8_t {
  PKT_ODOM = 0x01,
  PKT_IMU = 0x02,
  PKT_CMDVEL = 0x10
};

enum RxState { WAIT_SYNC1, WAIT_SYNC2, WAIT_TYPE, WAIT_LEN, WAIT_PAYLOAD, WAIT_CRC };

class UARTInterface {
  public:
    UARTInterface(HardwareSerial &serial);
    void begin(uint32_t baudrate);

    // Call this as fast as possible in loop()
    void update(); 

    // Returns true if a NEW packet is ready to be read
    bool isPacketAvailable();

    // Getters for the new packet data
    PacketType getPacketType();
    uint8_t getPacketLength();
    void getPacketPayload(uint8_t *out_buffer);

    // Send data back to Pi
    void sendPacket(PacketType type, const uint8_t *payload, uint8_t length);

  private:
    HardwareSerial &serial_;
    uint8_t computeCRC(const uint8_t *data, uint8_t length);

    RxState rx_state = WAIT_SYNC1;
    uint8_t rx_type;
    uint8_t rx_len;
    uint8_t rx_payload[64]; // Internal buffer
    uint8_t rx_index;
    uint8_t rx_crc;

    // Holding area for the completed packet available for main
    bool packet_ready_ = false;
    PacketType stored_type_;
    uint8_t stored_len_;
    uint8_t stored_payload_[64]; 
};
#endif