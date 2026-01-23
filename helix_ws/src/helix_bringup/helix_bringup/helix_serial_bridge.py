import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import serial
import struct
import time

# Packet Constants
SYNC_1 = 0xAA
SYNC_2 = 0x55
PKT_ODOM = 0x01
PKT_CMDVEL = 0x10

class HelixSerialBridge(Node):
    def __init__(self):
        super().__init__('helix_serial_bridge')
        
        # 1. Setup Serial Connection
        try:
            self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.1)
            self.get_logger().info(f"Connected to Teensy on {self.ser.port}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to Serial: {e}")
            raise e

        # 2. Setup ROS Publishers and Subscribers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.cmd_sub = self.create_subscription(
            Twist, 
            '/cmd_vel', 
            self.cmd_vel_callback, 
            10
        )

        # 3. Setup Read Timer
        self.create_timer(0.01, self.read_serial_data) # Check serial every 10ms

        # Buffer for parsing
        self.buffer = b''

    def cmd_vel_callback(self, msg: Twist):
        """Converts ROS Twist to Binary Packet and sends to Teensy"""
        # Pack float linear.x and angular.z into 8 bytes (little-endian) --> got from chatGPT
        payload = struct.pack('<ff', msg.linear.x, msg.angular.z)
        self.send_packet(PKT_CMDVEL, payload)

    def send_packet(self, pkt_type, payload):
        length = len(payload)
        crc = 0
        for byte in payload:
            crc ^= byte
        
        # Packet: [SYNC1, SYNC2, TYPE, LEN, PAYLOAD..., CRC]
        packet = struct.pack(f'<BBBB{length}sB', 
                             SYNC_1, SYNC_2, pkt_type, length, payload, crc)
        self.ser.write(packet)

    def read_serial_data(self):
        """Reads serial buffer and parses packets"""
        if self.ser.in_waiting > 0:
            self.buffer += self.ser.read(self.ser.in_waiting)
            self.process_buffer()

    def process_buffer(self):
        """State Machine"""
        while len(self.buffer) >= 5: # Min size: Sync1+Sync2+Type+Len+CRC
            # Find Sync bytes
            try:
                # Look for header 0xAA 0x55
                start_idx = self.buffer.index(b'\xaa\x55')
                
                # Discard noise before the header
                if start_idx > 0:
                    self.buffer = self.buffer[start_idx:]
                
                # Check if we have enough bytes for header logic
                if len(self.buffer) < 4:
                    return # Wait for more data

                pkt_type = self.buffer[2]
                payload_len = self.buffer[3]
                total_pkt_size = 4 + payload_len + 1 # Header(4) + Payload + CRC(1)

                if len(self.buffer) < total_pkt_size:
                    return # Wait for full packet

                # Extract Payload and CRC
                payload = self.buffer[4:4+payload_len]
                received_crc = self.buffer[4+payload_len]

                # Validate CRC
                calc_crc = 0
                for b in payload:
                    calc_crc ^= b
                
                if calc_crc == received_crc:
                    self.handle_packet(pkt_type, payload)
                    # Remove processed packet from buffer
                    self.buffer = self.buffer[total_pkt_size:]
                else:
                    self.get_logger().warn("CRC Mismatch! Dropping byte.")
                    self.buffer = self.buffer[1:] # Shift one byte to retry sync
                    
            except ValueError:
                # Sync bytes not found, keep last byte just in case it's SYNC1
                self.buffer = self.buffer[-1:] 

    def handle_packet(self, pkt_type, payload):
        if pkt_type == PKT_ODOM:
            self.publish_odom(payload)

    def publish_odom(self, payload):
        # Expected payload:
        # <timestamp (4)><left_ticks (4)><right_ticks (4)><lin_vel (4)><ang_vel (4)> = 20 bytes
        if len(payload) == 20:
            # Unpack: I=uint32 (time), i=int32 (ticks), f=float (vels)
            ts, l_tick, r_tick, lin_vel, ang_vel = struct.unpack('<Iiiff', payload)
            
            odom_msg = Odometry()
            odom_msg.header.stamp = self.get_clock().now().to_msg()
            odom_msg.header.frame_id = "odom"
            odom_msg.child_frame_id = "base_link"
            
            odom_msg.twist.twist.linear.x = lin_vel
            odom_msg.twist.twist.angular.z = ang_vel
            
            # Calculate pose (x, y, theta) here to fill odom_msg.pose.
            
            self.odom_pub.publish(odom_msg)

def main(args=None):
    rclpy.init(args=args)
    node = HelixSerialBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()