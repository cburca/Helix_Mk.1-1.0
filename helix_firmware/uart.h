
#ifndef UART_H
#define UART_H
#include <sys/_stdint.h>
#include <sys/types.h>
#include "HardwareSerial.h"
#include <Arduino.h>

//  [SYNC][TYPE][LEN][PAYLOAD][CRC]

//   Sync    ->   2 bytes ->   Packet Alignment
//   Type    ->   1 byte  ->   What data is inside
//   Len     ->   1 byte  ->   Payload Size
//   Payload ->   N bytes ->   Actual data
//   CRC     ->   1 byte  ->   Error Detection  


// ---- Packet Constants ---- //

#define UART_SYNC_1  0xAA
#define UART_SYNC_2  0x55

enum PacketType : uint8_t {
  PKT_ODOM = 0x01,
  PKT_IMU = 0x02,
  PKT_CMDVEL = 0x10
  //PKT_ESTOP = 0x11
  //PKT_LEDSTATE = 0x12
};

// THINGS NEEDED TO BE SENT IN ODOM PAYLOAD:
// <timestamp>
// <left_ticks>
// <right_ticks>
// <linear_velocity>
// <angular_velocity>

enum RxState {
  WAIT_SYNC1,
  WAIT_SYNC2,
  WAIT_TYPE,
  WAIT_LEN,
  WAIT_PAYLOAD,
  WAIT_CRC
};


class UARTInterface {

  public:
    UARTInterface(HardwareSerial &serial, uint32_t baud);

    void begin();
    void sendPacket(PacketType type, const uint8_t *payload, uint8_t length);

    void processIncomingByte(uint8_t byte);
    void handlePacket(uint8_t type, uint8_t *payload, uint8_t length);

  private:
    HardwareSerial &serial_;
    uint8_t computeCRC(const uint8_t *data, uint8_t length);

    RxState rx_state = WAIT_SYNC1;
    uint8_t rx_type;
    uint8_t rx_len;
    uint8_t rx_payload[64];
    uint8_t rx_index;
    uint8_t rx_crc;

};

#endif