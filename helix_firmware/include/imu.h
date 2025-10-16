// Initialize the UART (e.g., Serial2.begin(9600);).
// Read the incoming bytes into a buffer.
// Parse WitMotion’s packet format:
// - Header: 0x55 (sync byte)
// - Data type byte (0x51 accel, 0x52 gyro, 0x53 angle, etc.)
// - 8 data bytes Checksum
// Example: a 0x53 packet gives roll, pitch, yaw in hundredths of degrees.
// ROS Interface (future)
// - Teensy publishes IMU data over UART to the Pi.
// - ROS driver/subscriber translates it into sensor_msgs/Imu and geometry_msgs/Vector3 topics.


#ifndef IMU_H
#define IMU_H

#include <Arduino.h>

struct IMUData {
    float accelX, accelY, accelZ;  // m/s²
    float gyroX, gyroY, gyroZ;     // °/s
    float roll, pitch, yaw;        // degrees
};

class IMU {
public:
    IMU(HardwareSerial &serialPort);
    void begin(uint32_t baud = 9600);
    void update();
    IMUData getData() const;

private:
    HardwareSerial &imuSerial;
    IMUData data;

    static const uint8_t PACKET_HEADER = 0x55;
    static const uint8_t PACKET_LEN = 11;

    enum PacketType {
        ACCEL = 0x51,
        GYRO  = 0x52,
        ANGLE = 0x53
    };

    bool readPacket(uint8_t *buffer);
    bool validateChecksum(const uint8_t *buffer);
    void parsePacket(const uint8_t *buffer);
};

#endif
