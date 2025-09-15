#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.constants import S_TO_NS
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
import numpy as np
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math
from tf_transformations import quaternion_from_euler


class NoisyController(Node):

    def __init__(self):
        super().__init__("noisy_controller")
        # Parameters from the launch file for noisy odometry
        self.declare_parameter("wheel_radius", 0.033)
        self.declare_parameter("wheel_separation", 0.17)
        self.declare_parameter("publish_rate", 100.0)  # Hz

        # Exact values for clean odometry (independent of launch file)
        self.exact_wheel_radius = 0.033
        self.exact_wheel_separation = 0.17

        # Get parameters for noisy odometry
        self.wheel_radius_ = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.wheel_separation_ = self.get_parameter("wheel_separation").get_parameter_value().double_value
        self.publish_rate_ = self.get_parameter("publish_rate").get_parameter_value().double_value

        self.get_logger().info(f"Using wheel radius (noisy): {self.wheel_radius_}")
        self.get_logger().info(f"Using wheel separation (noisy): {self.wheel_separation_}")
        self.get_logger().info(f"Publishing rate: {self.publish_rate_} Hz")

        # Variables for noisy odometry
        self.left_wheel_prev_pos_ = 0.0
        self.right_wheel_prev_pos_ = 0.0
        self.x_ = 0.0
        self.y_ = 0.0
        self.theta_ = 0.0
        self.linear_velocity_ = 0.0
        self.angular_velocity_ = 0.0

        # Variables for clean odometry
        self.x_clean_ = 0.0
        self.y_clean_ = 0.0
        self.theta_clean_ = 0.0
        self.linear_velocity_clean_ = 0.0
        self.angular_velocity_clean_ = 0.0

        # Joint state subscription
        self.joint_sub_ = self.create_subscription(JointState, "joint_states", self.joint_callback, 10)

        # Publishers for noisy and clean odometry
        self.odom_pub_ = self.create_publisher(Odometry, "bumperbot_controller/odom_noisy", 10)
        self.clean_odom_pub_ = self.create_publisher(Odometry, "bumperbot_controller/odom_clean", 10)

        # Speed conversion matrix (for noisy odometry)
        self.speed_conversion_ = np.array([[self.wheel_radius_/2, self.wheel_radius_/2],
                                           [self.wheel_radius_/self.wheel_separation_, -self.wheel_radius_/self.wheel_separation_]])
        self.get_logger().info(f"The conversion matrix is {self.speed_conversion_}")

        # Odometry message setup for noisy odometry
        self.odom_msg_ = Odometry()
        self.odom_msg_.header.frame_id = "odom"
        self.odom_msg_.child_frame_id = "base_footprint_ekf"

        # Odometry message setup for clean odometry
        self.clean_odom_msg_ = Odometry()
        self.clean_odom_msg_.header.frame_id = "odom"
        self.clean_odom_msg_.child_frame_id = "base_footprint_clean"

        # Dynamic transform broadcaster setup
        self.br_dynamic = TransformBroadcaster(self)
        self.transform_stamped_noisy_to_base = TransformStamped()
        self.transform_stamped_noisy_to_base.header.frame_id = "base_footprint_ekf"
        self.transform_stamped_noisy_to_base.child_frame_id = "base_footprint"

        self.prev_time_ = self.get_clock().now()

        # Create a timer for publishing transforms and odometry
        self.timer = self.create_timer(1.0 / self.publish_rate_, self.timer_callback, callback_group=ReentrantCallbackGroup())

    def joint_callback(self, msg):
        try:
            # Add noise to wheel readings for noisy odometry
            wheel_encoder_left = msg.position[1] + np.random.normal(0, 0.005)
            wheel_encoder_right = msg.position[0] + np.random.normal(0, 0.005)

            dp_left = wheel_encoder_left - self.left_wheel_prev_pos_
            dp_right = wheel_encoder_right - self.right_wheel_prev_pos_
            dt = Time.from_msg(msg.header.stamp) - self.prev_time_

            # Update previous positions and time
            self.left_wheel_prev_pos_ = msg.position[1]
            self.right_wheel_prev_pos_ = msg.position[0]
            self.prev_time_ = Time.from_msg(msg.header.stamp)

            # Noisy odometry calculations
            fi_left = dp_left / (dt.nanoseconds / S_TO_NS)
            fi_right = dp_right / (dt.nanoseconds / S_TO_NS)
            self.linear_velocity_ = (self.wheel_radius_ * fi_right + self.wheel_radius_ * fi_left) / 2
            self.angular_velocity_ = (self.wheel_radius_ * fi_right - self.wheel_radius_ * fi_left) / self.wheel_separation_
            d_s = (self.wheel_radius_ * dp_right + self.wheel_radius_ * dp_left) / 2
            d_theta = (self.wheel_radius_ * dp_right - self.wheel_radius_ * dp_left) / self.wheel_separation_
            self.theta_ += d_theta
            self.x_ += d_s * math.cos(self.theta_)
            self.y_ += d_s * math.sin(self.theta_)

            # Clean odometry calculations (using exact values)
            fi_left_clean = dp_left / (dt.nanoseconds / S_TO_NS)
            fi_right_clean = dp_right / (dt.nanoseconds / S_TO_NS)
            self.linear_velocity_clean_ = (self.exact_wheel_radius * fi_right_clean + self.exact_wheel_radius * fi_left_clean) / 2
            self.angular_velocity_clean_ = (self.exact_wheel_radius * fi_right_clean - self.exact_wheel_radius * fi_left_clean) / self.exact_wheel_separation
            d_s_clean = (self.exact_wheel_radius * dp_right + self.exact_wheel_radius * dp_left) / 2
            d_theta_clean = (self.exact_wheel_radius * dp_right - self.exact_wheel_radius * dp_left) / self.exact_wheel_separation
            self.theta_clean_ += d_theta_clean
            self.x_clean_ += d_s_clean * math.cos(self.theta_clean_)
            self.y_clean_ += d_s_clean * math.sin(self.theta_clean_)

        except Exception as e:
            self.get_logger().error(f"Error in joint_callback: {str(e)}")

    def timer_callback(self):
        try:
            current_time = self.get_clock().now()

            # Noisy odometry message
            q_noisy = quaternion_from_euler(0, 0, self.theta_)
            self.odom_msg_.header.stamp = current_time.to_msg()
            self.odom_msg_.pose.pose.position.x = self.x_
            self.odom_msg_.pose.pose.position.y = self.y_
            self.odom_msg_.pose.pose.orientation.x = q_noisy[0]
            self.odom_msg_.pose.pose.orientation.y = q_noisy[1]
            self.odom_msg_.pose.pose.orientation.z = q_noisy[2]
            self.odom_msg_.pose.pose.orientation.w = q_noisy[3]
            self.odom_msg_.twist.twist.linear.x = self.linear_velocity_
            self.odom_msg_.twist.twist.angular.z = self.angular_velocity_

            # Clean odometry message
            q_clean = quaternion_from_euler(0, 0, self.theta_clean_)
            self.clean_odom_msg_.header.stamp = current_time.to_msg()
            self.clean_odom_msg_.pose.pose.position.x = self.x_clean_
            self.clean_odom_msg_.pose.pose.position.y = self.y_clean_
            self.clean_odom_msg_.pose.pose.orientation.x = q_clean[0]
            self.clean_odom_msg_.pose.pose.orientation.y = q_clean[1]
            self.clean_odom_msg_.pose.pose.orientation.z = q_clean[2]
            self.clean_odom_msg_.pose.pose.orientation.w = q_clean[3]
            self.clean_odom_msg_.twist.twist.linear.x = self.linear_velocity_clean_
            self.clean_odom_msg_.twist.twist.angular.z = self.angular_velocity_clean_

            # TF from base_footprint_noisy to base_footprint
            self.transform_stamped_noisy_to_base.header.stamp = current_time.to_msg()
            self.transform_stamped_noisy_to_base.transform.translation.x = 0.0
            self.transform_stamped_noisy_to_base.transform.translation.y = 0.0
            self.transform_stamped_noisy_to_base.transform.rotation.x = 0.0
            self.transform_stamped_noisy_to_base.transform.rotation.y = 0.0
            self.transform_stamped_noisy_to_base.transform.rotation.z = 0.0
            self.transform_stamped_noisy_to_base.transform.rotation.w = 1.0

            # Publish odometry and the transform
            self.odom_pub_.publish(self.odom_msg_)
            self.clean_odom_pub_.publish(self.clean_odom_msg_)
            self.br_dynamic.sendTransform(self.transform_stamped_noisy_to_base)

        except Exception as e:
            self.get_logger().error(f"Error in timer_callback: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    
    try:
        noisy_controller = NoisyController()
        executor = MultiThreadedExecutor()
        executor.add_node(noisy_controller)
        executor.spin()
    except Exception as e:
        print(f"An error occurred: {str(e)}")
    finally:
        if 'noisy_controller' in locals():
            noisy_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
