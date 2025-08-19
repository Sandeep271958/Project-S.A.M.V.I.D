import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import String  # You can change this to a custom message if needed


class SerialPublisher(Node):
    def __init__(self):
        super().__init__('serial_publisher')
        self.publisher_ = self.create_publisher(String, 'serial_data', 10)
        self.serial_port = '/dev/ttyUSB0'  # Replace with your serial port
        self.baud_rate = 115200  # Match the Arduino baud rate
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            self.get_logger().info(f"Connected to {self.serial_port} at {self.baud_rate} baud.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to {self.serial_port}: {e}")
            rclpy.shutdown()

        self.timer = self.create_timer(0.01, self.read_serial_data)  # 100 Hz

    def read_serial_data(self):
        try:
            if self.ser.in_waiting > 0:
                raw_data = self.ser.readline().decode('utf-8').strip()  
                if raw_data.startswith('<') and raw_data.endswith('>'):
                    self.publisher_.publish(String(data=raw_data))
                    self.get_logger().info(f"Published: {raw_data}")
                else:
                    self.get_logger().warn(f"Invalid data format: {raw_data}")
        except Exception as e:
            self.get_logger().error(f"Error reading serial data: {e}")


def main(args=None):
    rclpy.init(args=args)
    serial_publisher = SerialPublisher()
    rclpy.spin(serial_publisher)
    serial_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()