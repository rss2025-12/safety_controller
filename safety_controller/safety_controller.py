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
from std_msgs.msg import Float32, Bool, Int32
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
import math, csv, os, time


class SafetyController(Node):
    def __init__(self):
        super().__init__('safety_controller')

        self.VELOCITY = None
        self.STOP_COEFFICIENT = 0.8 # 0.65
        self.front_spread = 8
        # self.DECELERATION = 0.5
        self.csv_file = "safety_controller_data.csv"

        ### Publishers and subscribers ###
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.listener_callback, 10)
        self.ackerman_sub = self.create_subscription(AckermannDriveStamped, 'vesc/high_level/ackermann_cmd', self.acker_callback, 10)
        self.redlight_sub = self.create_subscription(Bool, '/redlight', self.redlight_callback, 10)
        self.traffic_light_sub = self.create_subscription(Bool, '/traffic_light_seen', self.traffic_light_callback, 1)
        self.publisher = self.create_publisher(AckermannDriveStamped, 'vesc/low_level/input/safety', 10)
        self.banana_id_sub = self.create_subscription(Int32, '/banana_id', self.banana_callback, 1)

        self.light_detector_red = False
        self.traffic_light_seen = False
        self.banana_id = 0

    def banana_callback(self, msg):
        self.banana_id = msg.data

    def acker_callback(self, msg):
        self.VELOCITY = msg.drive.speed

    def redlight_callback(self, msg):
        # self.get_logger().info(f'Traffic light msg data is {msg.data}')
        # if msg.data:
            # self.get_logger().info("Traffic light stop.")
        self.light_detector_red = msg.data
            # self.stop_vehicle()

    def traffic_light_callback(self, msg):
        self.traffic_light_seen = msg.data

    def is_red_light(self):
        # if self.light_detector_red and self.traffic_light_seen:
        if self.light_detector_red and self.banana_id < 1:
            return True
        else:
            return False

    def listener_callback(self, msg):
        def deg_to_index(deg):
            return int((deg * math.pi / 180 - angle_min) / angle_increment)

        if self.VELOCITY is None:
            return

        # Constants
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        ranges = msg.ranges
        stopping_distance = self.VELOCITY * self.STOP_COEFFICIENT
        # stopping_distance = (self.VELOCITY ** 2) / (2 * self.DECELERATION)

        # Logic
        front = ranges[deg_to_index(-self.front_spread):deg_to_index(self.front_spread)]
        if min(front) < stopping_distance:
            # self.safety_controller_data(self.csv_file, min(front))
            self.stop_vehicle()
            # self.get_logger().info("Safety stop.")
        if self.is_red_light():
            self.stop_vehicle()
            self.get_logger().info("Traffic light stop.")

    def stop_vehicle(self):
        acker = AckermannDriveStamped()
        acker.header.stamp = self.get_clock().now().to_msg()
        acker.drive.speed = 0.0
        acker.drive.acceleration = 0.0
        acker.drive.jerk = 0.0
        acker.drive.steering_angle = 0.0
        acker.drive.steering_angle_velocity = 0.0
        self.publisher.publish(acker)
        # self.get_logger().info("Safety stop.")


    # def safety_controller_data(self, csv_filename, distance, interval=0.5):
        # Check if the file already exists so we can write header only once.
        # file_exists = os.path.exists(csv_filename)

        # with open(csv_filename, 'a', newline='') as csvfile:
        #     writer = csv.writer(csvfile)
        #     # Write header if file is new or empty.
        #     if not file_exists:
        #         writer.writerow(["timestamp", "plot_distance"])

        #     # Get the current time stamp in a readable format.
        #     timestamp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())

        #     if (time.localtime().tm_sec % 1 == 0):
        #         # Write the new row to the CSV.
        #         writer.writerow([timestamp, distance])

            # Wait for the specified interval before the next log.
            # time.sleep(interval)


def main(args=None):
    rclpy.init(args=args)
    safety_controller = SafetyController()
    rclpy.spin(safety_controller)

    # Destroy the node explicitly
    safety_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
