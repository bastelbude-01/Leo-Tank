#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Int8

class JoystickController(Node):
    def __init__(self):
        super().__init__('joy_tank_node')
        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )
        self.servo_pos_pub = self.create_publisher(Int8, '/tank_msg', 10) #Servo_Pos
        self.servo_1_publisher = self.create_publisher(Int8, '/servo_pipe', 10)
        self.servo1_position = 0

    def joy_callback(self, msg):
        # Check if the message has enough buttons
        if len(msg.buttons) >= 4:
            button_A = msg.buttons[0]
            button_B = msg.buttons[1]
            button_X = msg.buttons[2]
            button_Y = msg.buttons[3]

            # Handle button presses and publish corresponding values on /LEDs topic
            if button_A == 1:
                self.publish_tank_msg(0)
            elif button_B == 1:
                self.publish_tank_msg(1)
            elif button_X == 1:
                self.publish_tank_msg(2)
            elif button_Y == 1:
                self.publish_tank_msg(3)

            # Handle servo control based on axes 6 and 7
            axis_6_value = msg.axes[6]
            axis_7_value = msg.axes[7]

            if axis_7_value == 1:
                self.servo1_position = 90
            elif axis_7_value == -1:
                self.servo1_position = 0
            elif axis_6_value == 1:
                self.servo1_position = min(self.servo1_position + 1, 125)
            elif axis_6_value == -1:
                self.servo1_position = max(self.servo1_position - 1, 0)

            # Publish the servo position
            servo_msg = Int8()
            servo_msg.data = self.servo1_position
            self.servo_1_publisher.publish(servo_msg)

    def publish_tank_msg(self, value):
        pos_msg = Int8()
        pos_msg.data = value
        self.servo_pos_pub.publish(pos_msg)

def main(args=None):
    rclpy.init(args=args)
    node = JoystickController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
