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


class SafetyController(Node):
    def __init__(self):
        super().__init__('safety_controller')

        self.VELOCITY = None
        self.STOP_COEFFICIENT = 0.6
        # self.DECELERATION = 0.5

        ### Publishers and subscribers ###
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.listener_callback, 10)
        self.ackerman_sub = self.create_subscription(AckermannDriveStamped, 'vesc/low_level/ackermann_cmd', self.acker_callback, 10)
        self.publisher = self.create_publisher(AckermannDriveStamped, 'vesc/low_level/input/safety', 10)

    def acker_callback(self, msg):
        self.VELOCITY = msg.drive.speed

    def listener_callback(self, msg):
        def deg_to_index(deg):
            return int((deg * math.pi / 180 - angle_min) / angle_increment)

        # Constants
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        ranges = msg.ranges
        front_spread = 8
        if self.VELOCITY is None:
            self.stop_vehicle()
            return
        stopping_distance = self.VELOCITY * self.STOP_COEFFICIENT
        # stopping_distance = (self.VELOCITY ** 2) / (2 * self.DECELERATION)

        # Logic
        front = ranges[deg_to_index(-front_spread): deg_to_index(front_spread)]
        if min(front) < stopping_distance:
            self.stop_vehicle()

    def stop_vehicle(self):
        acker = AckermannDriveStamped()
        acker.header.stamp = self.get_clock().now().to_msg()
        acker.drive.speed = 0.0
        acker.drive.acceleration = 0.0
        acker.drive.jerk = 0.0
        acker.drive.steering_angle = 0.0
        acker.drive.steering_angle_velocity = 0.0
        self.publisher.publish(acker)

        self.get_logger().info("Safety stop.")


def main(args=None):
    rclpy.init(args=args)
    safety_controller = SafetyController()
    rclpy.spin(safety_controller)

    # Destroy the node explicitly
    safety_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
