import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial
import struct

class BatteryMonitor(Node):
    def __init__(self):
        super().__init__('battery_monitor')
        self.publisher_ = self.create_publisher(Float32, 'battery_soc', 10)
        self.timer = self.create_timer(2.0, self.read_and_publish_soc)  # Set timer to call every 2 seconds
        self.serial_port = serial.Serial('/dev/ttyUSB0', baudrate=9600, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE)

    def read_and_publish_soc(self):
        # Construct and send packet as described in the previous steps
        packet = [0xA5, 0x40, 0x90, 8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        checksum = sum(packet) & 0xFF
        packet.append(checksum)
        self.serial_port.write(bytearray(packet))

        # Read and parse response
        response = self.serial_port.read(16)
        _, _, _, _, *data, _ = struct.unpack('BBBB8B', response)
        soc = (data[6] << 8 | data[7]) * 0.1

        # Publish SOC to the ROS 2 topic
        soc_msg = Float32()
        soc_msg.data = soc
        self.publisher_.publish(soc_msg)
        self.get_logger().info(f'Published SOC: {soc}%')

def main(args=None):
    rclpy.init(args=args)
    battery_monitor = BatteryMonitor()
    rclpy.spin(battery_monitor)
    battery_monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

