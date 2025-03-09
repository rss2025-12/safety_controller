# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
import math
import csv
import os
import time


class SafetyPublisher(Node):

    def __init__(self):
        super().__init__('safety_controller')

        self.subscriber = self.create_subscription(LaserScan, 'scan', self.listener_callback, 10)
        self.publisher = self.create_publisher(AckermannDriveStamped, 'vesc/low_level/input/safety', 10)
        self.safety_dist = 0.8
        self.front_spread = 5
        self.csv_file = "safety_controller_data.csv"

    def listener_callback(self, msg):
        def deg_to_index(deg):
            return int((deg * math.pi / 180 - angle_min) / angle_increment)

        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        ranges = msg.ranges
        mid_point = len(ranges) // 2
        front = ranges[deg_to_index(-self.front_spread):deg_to_index(self.front_spread)]

        if min(front) < self.safety_dist: # Ideally probably velocity
            acker = AckermannDriveStamped()
            acker.header.stamp = self.get_clock().now().to_msg()
            acker.drive.speed = 0.0
            acker.drive.acceleration = 0.0
            acker.drive.jerk = 0.0
            acker.drive.steering_angle = 0.0
            acker.drive.steering_angle_velocity = 0.0
            self.publisher.publish(acker)
            self.get_logger().info("Safety stop.")

            self.safety_controller_data(self.csv_file, min(front))
    
    def safety_controller_data(csv_filename, distance, interval=0.5):
        # Check if the file already exists so we can write header only once.
        file_exists = os.path.exists(csv_filename) and os.stat(csv_filename).st_size > 0
        
        with open(csv_filename, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            # Write header if file is new or empty.
            if not file_exists:
                writer.writerow(["timestamp", "plot_distance"])
            
            # Get the current time stamp in a readable format.
            timestamp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
            
            # Write the new row to the CSV.
            writer.writerow([timestamp, distance])
            
            # Wait for the specified interval before the next log.
            time.sleep(interval)


def main(args=None):
    rclpy.init(args=args)
    safety_controller = SafetyPublisher()
    rclpy.spin(safety_controller)

    # Destroy the node explicitly
    safety_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
