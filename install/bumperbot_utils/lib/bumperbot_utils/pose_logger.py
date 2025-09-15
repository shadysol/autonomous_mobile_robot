#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import csv
import os
from builtin_interfaces.msg import Time

class EnhancedPoseLogger(Node):
    def __init__(self):
        super().__init__('enhanced_pose_logger')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.1, self.log_pose)  # Log every 0.1 seconds
        
        self.csv_file = self.create_csv_file()
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['sim_time', 'odom_x', 'odom_y', 'slam_x', 'slam_y'])

    def log_pose(self):
        try:
            # Get current simulation time
            sim_time = self.get_clock().now().seconds_nanoseconds()
            sim_time_float = sim_time[0] + sim_time[1] / 1e9

            # Get the transform for odometry (odom -> base_footprint)
            t_odom = self.tf_buffer.lookup_transform('odom', 'base_footprint', rclpy.time.Time())

            # Get the transform for SLAM-estimated pose (map -> base_footprint)
            t_slam = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            
            # Log the data
            self.csv_writer.writerow([
                f"{sim_time_float:.6f}",
                f"{t_odom.transform.translation.x:.8f}",
                f"{t_odom.transform.translation.y:.8f}",
                f"{t_slam.transform.translation.x:.8f}",
                f"{t_slam.transform.translation.y:.8f}"
            ])
            self.csv_file.flush()  # Ensure data is written immediately
        except TransformException as ex:
            self.get_logger().info(f"Could not transform: {ex}")

    def create_csv_file(self):
        log_dir = os.path.join(os.getcwd(), 'log')
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)
        filepath = os.path.join(log_dir, 'enhanced_pose_log.csv')
        return open(filepath, 'w', newline='')

def main(args=None):
    rclpy.init(args=args)
    pose_logger = EnhancedPoseLogger()
    rclpy.spin(pose_logger)
    pose_logger.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()