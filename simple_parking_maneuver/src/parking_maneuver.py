#!/usr/bin/env python3
import rospy
from simple_parking_maneuver.srv import *
from simple_drive_control.srv import *


class SimpleParkingManeuver:

    def __init__(self):
        rospy.init_node("simple_parking_maneuver")

        self.driving_maneuver_client = rospy.ServiceProxy("driving_maneuver", DrivingManeuver)
        self.parking_service = rospy.Service("parking_maneuver", ParkingManeuver, self.parking_maneuver)

    def parking_maneuver(self, request):
        rospy.loginfo(rospy.get_caller_id() + ": callbackBackwardLongitudinal, direction = " + request.direction)

        # you can call the driving maneuver service like this
        # direction can be backward/forward, steering can be left/right/straight
        # self.driving_maneuver_client.call(direction="backward", steering="left", distance=0.3)

        if request.direction == "left":
            self.driving_maneuver_client.call(direction="backward", steering="left", distance=0.3)
            # TODO
        elif request.direction == "right":
            self.driving_maneuver_client.call(direction="backward", steering="left", distance=0.3)
            # TODO
        else:
            return ParkingManeuverResponse(
                "ERROR: Request can only be 'left' or 'right'")

        return ParkingManeuverResponse("FINISHED")


if __name__ == "__main__":
    SimpleParkingManeuver()
    rospy.spin()
