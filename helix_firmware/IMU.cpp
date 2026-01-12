#include "IMU.h"

IMU::IMU(HardwareSerial &serialPort) : imuSerial(serialPort) {
    data = {0, 0, 0, 0, 0, 0, 0, 0, 0};
}

void IMU::begin(uint32_t baud) {
    imuSerial.begin(baud);
}

// Reads and decodes any available packets
void IMU::update() {
    uint8_t buffer[PACKET_LEN];

    while (imuSerial.available() >= PACKET_LEN) {
        if (readPacket(buffer)) {
            if (validateChecksum(buffer)) {
                parsePacket(buffer);
            }
        }
    }
}

IMUData IMU::getData() const {
    return data;
}

// --- Internal helpers ---

bool IMU::readPacket(uint8_t *buffer) {
    // Sync to header byte 0x55
    if (imuSerial.read() != PACKET_HEADER) return false;

    // Read remaining bytes of packet
    buffer[0] = PACKET_HEADER;
    for (int i = 1; i < PACKET_LEN; i++) {
        int val = imuSerial.read();
        if (val == -1) return false;
        buffer[i] = (uint8_t)val;
    }
    return true;
}

bool IMU::validateChecksum(const uint8_t *buffer) {
    uint8_t sum = 0;
    for (int i = 0; i < PACKET_LEN - 1; i++) sum += buffer[i];
    return (sum & 0xFF) == buffer[PACKET_LEN - 1];
}

void IMU::parsePacket(const uint8_t *buffer) {
    uint8_t type = buffer[1];
    int16_t raw[3];

    // Convert raw bytes to signed 16-bit
    for (int i = 0; i < 3; i++) {
        raw[i] = (int16_t)(buffer[2 + i * 2] | (buffer[3 + i * 2] << 8));
    }

    switch (type) {
        case ACCEL:
            data.accelX = raw[0] / 32768.0f * 16.0f * 9.80665f; // m/s²
            data.accelY = raw[1] / 32768.0f * 16.0f * 9.80665f;
            data.accelZ = raw[2] / 32768.0f * 16.0f * 9.80665f;
            break;

        case GYRO:
            data.gyroX = raw[0] / 32768.0f * 2000.0f; // °/s
            data.gyroY = raw[1] / 32768.0f * 2000.0f;
            data.gyroZ = raw[2] / 32768.0f * 2000.0f;
            break;

        case ANGLE:
            data.roll  = raw[0] / 32768.0f * 180.0f;
            data.pitch = raw[1] / 32768.0f * 180.0f;
            data.yaw   = raw[2] / 32768.0f * 180.0f;
            break;

        default:
            break;
    }
}
