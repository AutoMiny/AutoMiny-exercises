#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from math import sqrt
from autominy_msgs.msg import SpeedCommand, NormalizedSteeringCommand
from simple_drive_control.srv import DrivingManeuver
from nav_msgs.msg import Odometry


class DriveControl(Node):
    def __init__(self):
        super().__init__("simple_drive_control")
        self.speed = 0.3  # m/s
        self.angle_left = 0.9
        self.angle_straight = 0.0
        self.angle_right = -0.9

        self.request_time = self.get_clock().now()  # to check for a timeout
        self.timeout = 30  # timeout after this amount of seconds
        self.distance = 0.0  # current driven distance
        self.odom = None  # current position
        self.active = False

        self.speed_pub = self.create_publisher(SpeedCommand, "actuators/speed", 1)
        self.steering_pub = self.create_publisher(NormalizedSteeringCommand, "actuators/steering_normalized", 1)
        self.odom_sub = self.create_subscription(Odometry, "simulation/odom_ground_truth", self.on_odom, 10)
        self.service_client = self.create_service(DrivingManeuver, "driving_maneuver", self.drive,
                                                  callback_group=MutuallyExclusiveCallbackGroup())

    # calculates the distance if maneuver is active
    def on_odom(self, msg):
        if self.odom is None or not self.active:
            self.odom = msg
            return

        self.distance += sqrt((self.odom.pose.pose.position.x - msg.pose.pose.position.x) ** 2 +
                              (self.odom.pose.pose.position.y - msg.pose.pose.position.y) ** 2)
        self.odom = msg

    # executes maneuver synchronously
    def drive(self, req, resp):
        self.request_time = self.get_clock().now()
        self.distance = 0
        self.active = True

        # parse and send steering command
        steering_cmd = NormalizedSteeringCommand()
        if req.steering == "left":
            steering_cmd.value = self.angle_left
        elif req.steering == "right":
            steering_cmd.value = self.angle_right
        elif req.steering == "straight":
            steering_cmd.value = self.angle_straight
        else:
            resp.success = False
            return resp
        self.steering_pub.publish(steering_cmd)

        # parse and send speed command
        if req.direction == "forward":
            direction = 1.0
        elif req.direction == "backward":
            direction = -1.0
        else:
            resp.success = False
            return resp

        speed_cmd = SpeedCommand()
        speed_cmd.value = self.speed * direction
        self.speed_pub.publish(speed_cmd)

        self.get_logger().info(f"Executing maneuver direction: {req.direction} steering: {req.steering} distance: {req.distance}m")

        # check if there is a timeout
        while rclpy.ok() and (self.get_clock().now() - self.request_time).nanoseconds / 1e9 < self.timeout:
            # wait until the car drove the desired distance and brake
            if self.distance >= req.distance:
                self.active = False
                speed_cmd.value = 0.0
                self.speed_pub.publish(speed_cmd)
                resp.success = True
                return resp  # indicates success
            time.sleep(0.01)

        # stop the car
        speed_cmd.value = 0.0
        self.speed_pub.publish(speed_cmd)
        self.get_logger().info("Timeout")
        resp.success = False
        return resp


def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor(num_threads=10)
    executor.add_node(DriveControl())
    executor.spin()
    executor.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
