#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.constants import S_TO_NS
from rclpy.time import Time
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
import numpy as np
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math
from tf_transformations import quaternion_from_euler


class MecanumController(Node):

    def __init__(self):
        super().__init__("mecanum_controller")
        self.declare_parameter("wheel_radius", 0.1)
        self.declare_parameter("wheel_separation_length", 0.35)
        self.declare_parameter("wheel_separation_width", 0.34)

        self.wheel_radius_ = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.wheel_separation_y = self.get_parameter("wheel_separation_length").get_parameter_value().double_value
        self.wheel_separation_x = self.get_parameter("wheel_separation_width").get_parameter_value().double_value

        self.get_logger().info("Using wheel radius %d" % self.wheel_radius_)
        self.get_logger().info("Using wheel separation length %d" % self.wheel_separation_y)
        self.get_logger().info("Using wheel separation width %d" % self.wheel_separation_x)

        self.left_wheel_front_prev_pos_ = 0.0
        self.right_wheel_front_prev_pos_ = 0.0
        self.left_wheel_back_prev_pos_ = 0.0
        self.right_wheel_back_prev_pos_ = 0.0
        self.x_ = 0.0
        self.y_ = 0.0
        self.theta_ = 0.0

        self.wheel_cmd_pub_ = self.create_publisher(Float64MultiArray, "mecanum_controller/commands", 10)
        self.vel_sub_ = self.create_subscription(TwistStamped, "zeta_controller/cmd_vel", self.velCallback, 10)
        self.joint_sub_ = self.create_subscription(JointState,"joint_states", self.jointCallback, 10)        
        self.odom_pub_ = self.create_publisher(Odometry, "zeta_controller/odom", 10)

        self.speed_conversion_ = np.array([
            [1, -1, -(self.wheel_separation_x + self.wheel_separation_y) / self.wheel_radius_],
            [1,  1,  (self.wheel_separation_x + self.wheel_separation_y) / self.wheel_radius_],
            [1,  1, -(self.wheel_separation_x + self.wheel_separation_y) / self.wheel_radius_],
            [1, -1,  (self.wheel_separation_x + self.wheel_separation_y) / self.wheel_radius_]
        ])
        self.get_logger().info("The conversion matrix is %s" % self.speed_conversion_)

        # Fill the Odometry message with invariant parameters
        self.odom_msg_ = Odometry()
        self.odom_msg_.header.frame_id = "odom"
        self.odom_msg_.child_frame_id = "base_footprint"
        self.odom_msg_.pose.pose.orientation.x = 0.0
        self.odom_msg_.pose.pose.orientation.y = 0.0
        self.odom_msg_.pose.pose.orientation.z = 0.0
        self.odom_msg_.pose.pose.orientation.w = 1.0

        # Fill the TF message
        self.br_ = TransformBroadcaster(self)
        self.transform_stamped_ = TransformStamped()
        self.transform_stamped_.header.frame_id = "odom"
        self.transform_stamped_.child_frame_id = "base_footprint"

        self.prev_time_ = self.get_clock().now()


    def velCallback(self, msg):
        # Implements the mecanum kinematic model
        # Given v_x, v_y, and omega, calculate the velocities of the wheels
        robot_speed = np.array([[msg.twist.linear.x], [msg.twist.linear.y], [msg.twist.angular.z]])
        wheel_speed = np.dot(self.speed_conversion_, robot_speed)
        wheel_speed_msg = Float64MultiArray()
        wheel_speed_msg.data = wheel_speed.flatten().tolist()

        self.wheel_cmd_pub_.publish(wheel_speed_msg)

    
    def jointCallback(self, msg):
        # Implements the inverse differential kinematic model
        # Given the position of the wheels, calculates their velocities
        # then calculates the velocity of the robot wrt the robot frame
        # and then converts it in the global frame and publishes the TF
        dp_front_left = msg.position[0] - self.left_wheel_front_prev_pos_
        dp_front_right = msg.position[1] - self.right_wheel_front_prev_pos_
        dp_back_left = msg.position[2] - self.left_wheel_back_prev_pos_
        dp_back_right = msg.position[3] - self.right_wheel_back_prev_pos_

        dt = (Time.from_msg(msg.header.stamp) - self.prev_time_).nanoseconds / 1e9

        # Actualize the prev pose for the next itheration
        self.left_wheel_front_prev_pos_ = msg.position[0]
        self.right_wheel_front_prev_pos_ = msg.position[1]
        self.left_wheel_back_prev_pos_ = msg.position[2]
        self.right_wheel_back_prev_pos_ = msg.position[3]
        self.prev_time_ = Time.from_msg(msg.header.stamp)

        # Calculate the rotational speed of each wheel
        fi_front_left = dp_front_left / dt
        fi_front_right = dp_front_right / dt
        fi_back_left = dp_back_left / dt
        fi_back_right = dp_back_right / dt

        # Inverse kinematic model
        wheel_speeds = np.array([fi_front_left, fi_front_right, fi_back_left, fi_back_right]).reshape(4, 1)
        inverse_speed_conversion = (self.wheel_radius_ / 4) * np.array([
            [1,  1,  1,  1],
            [-1, 1,  1, -1],
            [-(1 / (self.wheel_separation_x + self.wheel_separation_y)),
             (1 / (self.wheel_separation_x + self.wheel_separation_y)),
             -(1 / (self.wheel_separation_x + self.wheel_separation_y)),
             (1 / (self.wheel_separation_x + self.wheel_separation_y))]
        ])

        robot_speed = np.dot(inverse_speed_conversion, wheel_speeds)
        linear_x = robot_speed[0, 0]
        linear_y = robot_speed[1, 0]
        angular_z = robot_speed[2, 0]

        # Update robot pose
        self.theta_ += angular_z * dt
        self.x_ += (linear_x * math.cos(self.theta_) - linear_y * math.sin(self.theta_)) * dt
        self.y_ += (linear_x * math.sin(self.theta_) + linear_y * math.cos(self.theta_)) * dt

        # Update odometry
        q = quaternion_from_euler(0, 0, self.theta_)
        self.odom_msg_.header.stamp = self.get_clock().now().to_msg()
        self.odom_msg_.pose.pose.position.x = self.x_
        self.odom_msg_.pose.pose.position.y = self.y_
        self.odom_msg_.pose.pose.orientation.x = q[0]
        self.odom_msg_.pose.pose.orientation.y = q[1]
        self.odom_msg_.pose.pose.orientation.z = q[2]
        self.odom_msg_.pose.pose.orientation.w = q[3]
        self.odom_msg_.twist.twist.linear.x = linear_x
        self.odom_msg_.twist.twist.linear.y = linear_y
        self.odom_msg_.twist.twist.angular.z = angular_z
        self.odom_pub_.publish(self.odom_msg_)

        # Update TF
        self.transform_stamped_.transform.translation.x = self.x_
        self.transform_stamped_.transform.translation.y = self.y_
        self.transform_stamped_.transform.rotation.x = q[0]
        self.transform_stamped_.transform.rotation.y = q[1]
        self.transform_stamped_.transform.rotation.z = q[2]
        self.transform_stamped_.transform.rotation.w = q[3]
        self.transform_stamped_.header.stamp = self.get_clock().now().to_msg()
        self.br_.sendTransform(self.transform_stamped_)


def main():
    rclpy.init()
    mecanum_controller = MecanumController()
    rclpy.spin(mecanum_controller)
    mecanum_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()