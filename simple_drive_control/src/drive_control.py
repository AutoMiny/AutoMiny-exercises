#!/usr/bin/env python3
import rospy
from math import sqrt
from autominy_msgs.msg import SpeedCommand, NormalizedSteeringCommand
from simple_drive_control.srv import DrivingManeuver
from nav_msgs.msg import Odometry


class DriveControl:
    def __init__(self):
        self.speed = 0.3  # m/s
        self.angle_left = 0.9
        self.angle_straight = 0.0
        self.angle_right = -0.9

        self.request_time = rospy.Time()  # to check for a timeout
        self.timeout = 30  # timeout after this amount of seconds
        self.distance = 0.0  # current driven distance
        self.odom = None  # current position
        self.active = False

        rospy.init_node("simple_drive_control")
        self.speed_pub = rospy.Publisher("actuators/speed", SpeedCommand, queue_size=1)
        self.steering_pub = rospy.Publisher("actuators/steering_normalized", NormalizedSteeringCommand, queue_size=1)
        self.odom_sub = rospy.Subscriber("sensors/localization/filtered_map", Odometry, self.on_odom, queue_size=10)
        self.service_client = rospy.Service("driving_maneuver", DrivingManeuver, self.drive, buff_size=1)

    # calculates the distance if maneuver is active
    def on_odom(self, msg):
        if self.odom is None and not self.active:
            self.odom = msg
            return

        self.distance += sqrt((self.odom.pose.pose.position.x - msg.pose.pose.position.x) ** 2 +
                              (self.odom.pose.pose.position.y - msg.pose.pose.position.y) ** 2)
        self.odom = msg

    # executes maneuver synchronously
    def drive(self, req):
        self.request_time = rospy.Time.now()
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
            return False
        self.steering_pub.publish(steering_cmd)

        # parse and send speed command
        if req.direction == "forward":
            direction = 1.0
        elif req.direction == "backward":
            direction = -1.0
        else:
            return False

        speed_cmd = SpeedCommand()
        speed_cmd.value = self.speed * direction
        self.speed_pub.publish(speed_cmd)

        # check if there is a timeout
        while not rospy.is_shutdown() and (rospy.Time.now() - self.request_time).to_sec() < self.timeout:
            # wait until the car drove the desired distance and brake
            if self.distance >= req.distance:
                self.active = False
                speed_cmd.value = 0
                self.speed_pub.publish(speed_cmd)
                return True  # indicates success
            rospy.sleep(0.01)

        # stop the car
        speed_cmd.value = 0
        self.speed_pub.publish(speed_cmd)

        return False


if __name__ == "__main__":
    DriveControl()
    rospy.spin()
