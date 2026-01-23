#include "uart.h"

UARTInterface::UARTInterface(HardwareSerial &serial) : serial_(serial) {}

void UARTInterface::begin(uint32_t baudrate){
  serial_.begin(baudrate);
}

// Check Serial buffer and update state machine
void UARTInterface::update() {
  while (serial_.available()) {
    uint8_t byte = serial_.read();
    
    switch (rx_state) {
      case WAIT_SYNC1:
        if (byte == UART_SYNC_1) rx_state = WAIT_SYNC2;
        break;

      case WAIT_SYNC2:
        if (byte == UART_SYNC_2) rx_state = WAIT_TYPE;
        else rx_state = WAIT_SYNC1;
        break;

      case WAIT_TYPE:
        rx_type = byte;
        rx_state = WAIT_LEN;
        break;

      case WAIT_LEN:
        rx_len = byte;
        rx_index = 0;
        if (rx_len > sizeof(rx_payload)) rx_state = WAIT_SYNC1; // Safety
        else rx_state = WAIT_PAYLOAD;
        break;

      case WAIT_PAYLOAD:
        rx_payload[rx_index++] = byte;
        if (rx_index >= rx_len) rx_state = WAIT_CRC;
        break;
      
      case WAIT_CRC:
        rx_crc = byte;
        if (computeCRC(rx_payload, rx_len) == rx_crc) {
          // Valid packet! Copy to storage for main loop
          stored_type_ = (PacketType)rx_type;
          stored_len_ = rx_len;
          memcpy(stored_payload_, rx_payload, rx_len);
          packet_ready_ = true;
        }
        rx_state = WAIT_SYNC1;
        break;
    }
  }
}

bool UARTInterface::isPacketAvailable() {
  if (packet_ready_) {
    packet_ready_ = false; // Clear flag on read
    return true;
  }
  return false;
}

PacketType UARTInterface::getPacketType() { return stored_type_; }
uint8_t UARTInterface::getPacketLength() { return stored_len_; }

void UARTInterface::getPacketPayload(uint8_t *out_buffer) {
  memcpy(out_buffer, stored_payload_, stored_len_);
}

void UARTInterface::sendPacket(PacketType type, const uint8_t *payload, uint8_t length){
  serial_.write(UART_SYNC_1);
  serial_.write(UART_SYNC_2);
  serial_.write((uint8_t)type);
  serial_.write(length);
  serial_.write(payload, length);
  serial_.write(computeCRC(payload, length));
}

uint8_t UARTInterface::computeCRC(const uint8_t *data, uint8_t length){
  uint8_t crc = 0;
  for(uint8_t i = 0; i < length; i++) crc ^= data[i];
  return crc;
}