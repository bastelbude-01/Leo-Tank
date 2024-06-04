#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState
import numpy as np
from rclpy.time import Time
from rclpy.constants import S_TO_NS


class TankController(Node):
    def __init__(self):
        super().__init__("tank_controller")

        self.declare_parameter("wheel_radius", 0.035)
        self.declare_parameter("wheel_seperation", 0.163)

        self.wheel_radius_ = self.get_parameter(
            "wheel_radius").get_parameter_value().double_value
        self.wheel_seperation_ = self.get_parameter(
            "wheel_seperation").get_parameter_value().double_value
        
        self.get_logger().info("Benutze Reifen Radius %f" % self.wheel_radius_)
        self.get_logger().info("Benutze Ketten Abstand %f" % self.wheel_seperation_)

        self.left_wheel_prev_pos_ = 0.0
        self.right_wheel_prev_pos_ = 0.0

        self.prev_time_ = self.get_clock().now()

        self.wheel_cmd_pub_ = self.create_publisher(
            Float64MultiArray, "tank_speed_controller/commands", 10)
        
        self.velocity_sub_ = self.create_subscription(
            TwistStamped, "tank_controller/cmd_vel", self.velocityCallback, 10)        

        self.joint_sub_ = self.create_subscription(
            JointState, "joint_states", self.jointCallback, 10)

        self.speed_conversion_ = np.array([[self.wheel_radius_/2, self.wheel_radius_/2],
                                           [self.wheel_radius_/self.wheel_seperation_, -self.wheel_radius_/self.wheel_seperation_]])
        

        self.get_logger().info("Die Converion Matrix ist %s" % self.speed_conversion_)

    def velocityCallback(self, msg):
        robot_speed = np.array([[msg.twist.linear.x],
                                [msg.twist.angular.z]])

        wheel_speed = np.matmul(np.linalg.inv(
            self.speed_conversion_), robot_speed)

        wheel_speed_msg = Float64MultiArray()
        wheel_speed_msg.data = [wheel_speed[1, 0], wheel_speed[0, 0]]
        self.wheel_cmd_pub_.publish(wheel_speed_msg)


    def jointCallback(self, msg):
        dp_left = msg.position[1] - self.left_wheel_prev_pos_
        dp_right = msg.position[0] - self.right_wheel_prev_pos_

        dt = Time.from_msg(msg.header.stamp) - self.prev_time_

        self.left_wheel_prev_pos_ = msg.position[1]
        self.right_wheel_prev_pos_ = msg.position[0]

        self.prev_time_ = Time.from_msg(msg.header.stamp)

        fi_left = dp_left / (dt.nanoseconds / S_TO_NS)
        fi_right = dp_right / (dt.nanoseconds / S_TO_NS)

        linear = (self.wheel_radius_ * fi_right +
                  self.wheel_radius_ * fi_left) / 2
        angular = (self.wheel_radius_ * fi_right -
                   self.wheel_radius_ * fi_left) / self.wheel_seperation_

        #self.get_logger().info("Linear: %f , Angular: %f" % (linear, angular))


def main():
    rclpy.init()
    simple_controller = TankController()
    rclpy.spin(simple_controller)
    simple_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
