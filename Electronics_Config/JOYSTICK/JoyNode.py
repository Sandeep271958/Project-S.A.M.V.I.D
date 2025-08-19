

import rclpy
from rclpy.node import Node
from encoder_interface.msg import AngularVel
from sensor_msgs.msg import Joy
import serial
import time
from std_msgs.msg import Int8

class EncoderDataNode(Node):
    def __init__(self):
        super().__init__("Encoder_data_publisher")

        self.encoder_data_publisher_ = self.create_publisher(AngularVel, "ang_vel", 10)

        self.ang_vel_subscriber_ = self.create_subscription(AngularVel, "cmd_vel", self.ang_vel_callback, 10)
        self.joy_subscriber_ = self.create_subscription(Joy, "/joy", self.joy_callback, 10)
        self.op_mode_subscriber_ = self.create_subscription(Int8 , "op_mode" , self.op_mode_callback , 10)
        self.e_stop_subscriber_ = self.create_subscription(Int8 , "e_stop" , self.e_stop_callback , 10)
        

        self.op_mode = 1
        self.e_stop = 0
        self.ser = serial.Serial('/dev/cytron_urc10', 115200, timeout=1)

        self.joy_data = Joy()
        self.ang_vel_data = AngularVel()

        self.button_pressed_time = None
        self.button_hold_threshold = 3.0  

    def joy_callback(self, msg):
        pass

    def op_mode_callback(self , msg):
        self.op_mode = msg.data

    def e_stop_callback(self , msg):
        self.e_stop = msg.data

    def ang_vel_callback(self, msg):
        self.ang_vel_data = msg

    def send_ang_vel(self):
        if self.op_mode:
            if len(self.joy_data.axes) >= 2:
                if (self.joy_data.axes[2] <= -0.2 or self.joy_data.axes[5] <= -0.2 or self.joy_data.buttons[4] == 1 or self.joy_data.buttons[5] == 1 or self.e_stop == 1):
                    ang_vel_0 = f"{self.joy_data.axes[1] * 0:.2f}"
                    ang_vel_1 = f"{self.joy_data.axes[3] * 0:.2f}"                                   
                else:
                    if (abs(self.joy_data.axes[1]) > 0.05 or abs(self.joy_data.axes[3]) > 0.05):
                        ang_vel_0 = f"{self.joy_data.axes[1] * 0.77:.2f}" # maximum PWM is 68 , to be changed HERE
                        ang_vel_1 = f"{self.joy_data.axes[3] * 3.66:.2f}"  # maximum PWM is 68 , to be changed HERE
                    else:
                        ang_vel_0 = f"{self.joy_data.axes[6] * 0.4 * 0.77:.2f}" # maximum PWM is 68 , to be changed HERE
                        ang_vel_1 = f"{self.joy_data.axes[7] * 0.4 * 3.66:.2f}"  # maximum PWM is 68 , to be changed HERE


                formatted_data = f"<{ang_vel_0},{ang_vel_1}>\n"
                self.get_logger().info(f'Sending to Arduino (Manual): "{formatted_data.strip()}"')
                self.ser.write(formatted_data.encode('utf-8'))
            else:
                self.get_logger().warn("Joystick data has insufficient axes.")
        else:
            if len(self.ang_vel_data.ang_vel) >= 2:
                ang_vel_0 = f"{self.ang_vel_data.ang_vel[0]:.2f}"   # maximum PWM is 68 , to be changed HERE
                ang_vel_1 = f"{self.ang_vel_data.ang_vel[1]:.2f}"   # maximum PWM is 68 , to be changed HERE
                formatted_data = f"<{ang_vel_0},{ang_vel_1}>"
                self.get_logger().info(f'Sending to Arduino (Automatic): "{formatted_data.strip()}"')
                self.ser.write(formatted_data.encode('utf-8'))
            else:
                self.get_logger().warn("Received an AngularVel message with insufficient data.")

    def encoder_send(self):
        if self.ser.in_waiting > 0:
            try:
                line = (self.ser.readline().decode('utf-8').strip())                
                if line.count("<")==1 and line.count(">")==1: #match:
                    vals = (line.replace("<","")).replace(">","").split(",")
                    left_count = float(vals[0])
                    right_count = float(vals[1])
                    
                    msg = AngularVel()
                    msg.ang_vel = [left_count , right_count]
                    self.encoder_data_publisher_.publish(msg)

                else:
                    #self.get_logger().warn(f"Received unexpected data: not execption")
                    pass

            except Exception as e:
                self.get_logger().warn(f"{e}")

def main(args=None):
    rclpy.init(args=args)
    node = EncoderDataNode()
    try:
        while rclpy.ok():
            node.send_ang_vel()
            node.encoder_send()
            rclpy.spin_once(node, timeout_sec=0.01)
            time.sleep(0.01)
    finally:
        node.destroy_node()
        node.get_logger().warn(f"Received unexpected data:")
    rclpy.shutdown()
