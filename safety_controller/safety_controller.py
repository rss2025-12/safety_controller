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


class SafetyPublisher(Node):

    def __init__(self):
        super().__init__('safety_controller')

        self.subscriber = self.create_subscription(LaserScan, 'scan', self.listener_callback, 10)
        # self.subscriber: skill issue
        self.publisher = self.create_publisher(AckermannDriveStamped, 'vesc/low_level/input/safety', 10)

    def listener_callback(self, msg):
        ranges = msg.ranges
        mid_point = len(ranges) // 2
        front_spread = 5
        front = ranges[mid_point - front_spread: mid_point + front_spread] # TODO: check which angels they correlate to

        if min(front) < 0.5: # Ideally probably velocity
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
    safety_controller = SafetyPublisher()
    rclpy.spin(safety_controller)

    # Destroy the node explicitly
    safety_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
