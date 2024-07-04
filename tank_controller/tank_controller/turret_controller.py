#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState
import numpy as np
from rclpy.time import Time
from rclpy.constants import S_TO_NS


class TurretController(Node):
    def __init__(self):
        super().__init__("turret_controller")

        self.declare_parameter("coil_radius", 0.06)
        self.declare_parameter("coil_seperation", 0.045)

        self.coil_radius = self.get_parameter("coil_radius").get_parameter_value().double_value
        self.coil_seperation = self.get_parameter("coil_seperation").get_parameter_value().double_value

        self.coil_prev_pos_ = 0.0

        self.prev_time_ = self.get_clock().now()

        self.turret_cmd_pub_ = self.create_publisher(Float64MultiArray, "turret_controller/commands", 10)
        self.turret_cmd_sub_ = self.create_subscription(TwistStamped, "turret_controller/cmd_vel", self.turretCallback, 10)
        self.joint_sub_ = self.create_subscription(JointState, "joint_states", self.jointCallback, 10)

        self.turret_conversion_ = self.coil_radius / self.coil_seperation

        self.coil_point = 0.0

    def turretCallback(self, msg):
        turm_speed = np.array([[msg.twist.linear.x]])
        coil_speed = turm_speed[1, 0]

        if abs(coil_speed) > 0.2:
            self.coil_point += 0.01
            turret_speed = self.turret_conversion_ / self.coil_point
        elif abs(coil_speed) < -0.2:
            self.coil_point -= 0.01
            turret_speed = self.turret_conversion_ / self.coil_point
        else:
            turret_speed = 0.0  

        
        turret_msg = Float64MultiArray()
        turret_msg.data = [turret_speed]
        self.turret_cmd_pub_.publish(turret_msg)

    def jointCallback(self, msg):
        dp_coil = msg.position[2] - self.coil_prev_pos_

        dt = Time.from_msg(msg.header.stamp) - self.prev_time_

        self.coil_prev_pos_ = msg.position[2]

        self.prev_time_ = Time.from_msg(msg.header.stamp)

        fi_coil = dp_coil / (dt.nanoseconds / S_TO_NS)


def main():
    rclpy.init()
    simple_controller = TurretController()
    rclpy.spin(simple_controller)
    simple_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()