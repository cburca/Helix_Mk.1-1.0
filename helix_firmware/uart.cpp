
/////////// IMPROVEMENTS /////////// 
// What happens if UART drops?
// - Timeout /cmd_vel
// - Zero motors if no commands received
// - Watchdog timeout logic inside Arduino


#include "WString.h"
#include "HardwareSerial.h"
#include "uart.h"

UARTInterface::UARTInterface(HardwareSerial &serial)
  : serial_(serial) {}


void UARTInterface::begin(uint32_t baudrate){
  serial_.begin(baudrate);
}


void UARTInterface::sendPacket(PacketType type, const uint8_t *payload, uint8_t length){
  serial_.write(UART_SYNC_1);
  serial_.write(UART_SYNC_2);

  serial_.write((uint8_t)type);
  serial_.write(length);

  serial_.write(payload, length);

  uint8_t crc = computeCRC(payload, length)

}


// Example implementation:
// uart.processIncomingByte(serial_).read()); -> inside infinite loop while serial channel is avaiable!

void UARTInterface::processIncomingByte(uint8_t byte){

  switch (byte) {
    case WAIT_SYNC1:
      if(byte == UART_SYNC_1){
        rx_state = WAIT_SYNC2;
      }
      break;

    case WAIT_SYNC2:
      if(byte == UART_SYNC_2){
        rx_state = WAIT_TYPE;
      } else {
        rx_state = WAIT_SYNC1;
      }
      break;

    case WAIT_TYPE:
      rx_type = byte;
      rx_state = WAIT_LEN;
      break;

    case WAIT_LEN:
      rx_len = byte;
      rx_index = 0;

      if(rx_len > sizeof(rx_payload)){
        rx_len = 0xff; // Too big - can maybe use for error check later?
        rx_state = WAIT_SYNC1;
      } else {
        rx_state = WAIT_PAYLOAD;
      }
      break;

    case WAIT_PAYLOAD:
      rx_payload[rx_index++] = byte;

      if(rx_index >= rx_len){
        rx_state = WAIT_CRC;
      }
      break;
    
    case WAIT_CRC:
      rx_crc = byte;

      uint8_t computed = computeCRC(rx_payload, rx_len);

      if(computed == rx_crc) {
        handlePacket(rx_type, rx_payload, rx_len);
      }

      rx_state = WAIT_SYNC1;
      break;
  }
}


void UARTInterface::handlePacket(uint8_t type, uint8_t *payload, uint8_t length) {
  switch (type) {

    case PKT_CMDVEL:
      if (length == 8) {
        float lin, ang;
        memcpy(&lin, &payload[0], 4);
        memcpy(&ang, &payload[4], 4);

        setTargetVelocity(lin, ang);
      }
      break;
  }
}


uint8_t UARTInterface::computeCRC(const uint8_t *data, uint8_t length){
  unit8_t CRC = 0;

  for(unit8_t i = 0; i < length; i++){
    crc ^= data[i];
  }

  return CRC;
}