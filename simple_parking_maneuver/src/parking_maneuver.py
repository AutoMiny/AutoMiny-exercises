#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from simple_parking_maneuver.srv import *
from simple_drive_control.srv import *

class SimpleParkingManeuver(Node):

    def __init__(self):
        super().__init__("simple_parking_maneuver")

        self.driving_maneuver_client = self.create_client(DrivingManeuver, "driving_maneuver")
        self.parking_service = self.create_service(ParkingManeuver, "parking_maneuver", self.parking_maneuver,
                                                   callback_group=MutuallyExclusiveCallbackGroup())

    def parking_maneuver(self, request, response):
        self.get_logger().info(f"executing parking maneuver direction: {request.direction}")

        # you can call the driving maneuver service like this
        # direction can be backward/forward, steering can be left/right/straight
        if request.direction == "left":
            self.driving_maneuver_client.call(DrivingManeuver.Request(direction="backward", steering="left", distance=0.3))
            self.driving_maneuver_client.call(DrivingManeuver.Request(direction="backward", steering="right", distance=0.3))
            self.driving_maneuver_client.call(DrivingManeuver.Request(direction="backward", steering="left", distance=0.3))
            self.driving_maneuver_client.call(DrivingManeuver.Request(direction="backward", steering="right", distance=0.3))
            # TODO
        elif request.direction == "right":
            self.driving_maneuver_client.call(DrivingManeuver.Request(direction="backward", steering="left", distance=0.3))
            # TODO
        else:
            response.info = "ERROR: Request can only be 'left' or 'right'"
            return response

        response.info = "FINISHED"
        return response


def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor(num_threads=10)
    executor.add_node(SimpleParkingManeuver())
    executor.spin()
    executor.shutdown()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
