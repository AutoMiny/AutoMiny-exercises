import rospy
from nav_msgs.msg import Odometry
from autominy_msgs.msg import SpeedCommand, SteeringCommand
from std_msgs.msg import UInt8
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from map import Map
import numpy as np
import math
import tf.transformations


class Navigation:

    def __init__(self):
        rospy.init_node("navigation")
        self.speed_pub = rospy.Publisher("/control/speed", SpeedCommand, queue_size=10)
        self.steering_pub = rospy.Publisher("/control/steering", SteeringCommand, queue_size=10)
        self.lookahead_pub = rospy.Publisher("lookahead", Marker, queue_size=10)

        self.localization_sub = rospy.Subscriber("/sensors/localization/filtered_map", Odometry, self.on_localization,
                                                 queue_size=1)
        self.lane_sub = rospy.Subscriber("lane", UInt8, self.on_lane, queue_size=1)
        self.scan_sub = rospy.Subscriber("/sensors/rplidar/scan", LaserScan, self.on_scan, queue_size=1)

        self.map = Map()
        self.rate = rospy.Rate(10)
        self.timer = rospy.Timer(rospy.Duration.from_sec(0.1), self.on_control)

        self.lane = 1

        self.pose = Odometry()
        self.desired_angle = 0.0

        rospy.on_shutdown(self.on_shutdown)

        while not rospy.is_shutdown():
            self.rate.sleep()

    def on_localization(self, msg):
        self.pose = msg

    def on_steering(self, msg):
        self.desired_angle = msg.value

    def on_lane(self, msg):
        self.lane = msg.data

    def on_scan(self, msg):
        # 10 deg
        points_in_front = int(0.25 / msg.angle_increment)

        angle = msg.angle_min
        i = 0
        points = np.empty([4, 0])
        for r in msg.ranges:
            angle += msg.angle_increment

            if r < 1.3 and (i < points_in_front or i > len(msg.ranges) - points_in_front):
                p = np.array([r * np.cos(angle) - 0.18, r * np.sin(angle), 0.0, 1.0])
                points = np.column_stack([points, p])

            i += 1

        # apply transformation into map frame (rotation around -yaw)
        quat = [self.pose.pose.pose.orientation.x, self.pose.pose.pose.orientation.y, self.pose.pose.pose.orientation.z,
                self.pose.pose.pose.orientation.w]
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(quat)

        # 180 deg rot
        yaw += np.pi

        t = np.array([
            [np.cos(yaw), -np.sin(yaw), 0.0, self.pose.pose.pose.position.x],
            [np.sin(yaw), np.cos(yaw), 0.0, self.pose.pose.pose.position.y],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]
        ])

        points = np.matmul(t, points)
        points = points[[0, 1], :]

        lane0_occupied = self.lane_occupied(self.map.lanes[0], points)
        lane1_occupied = self.lane_occupied(self.map.lanes[1], points)

        if self.lane == -1:
            if not lane1_occupied:
                self.lane = 1
            if not lane0_occupied:
                self.lane = 0

        if self.lane == 1 and lane1_occupied:
            print "lane switch to 0"
            self.lane = 0

        if self.lane == 0 and lane0_occupied:
            print "lane switch to 1"
            self.lane = 1

        if lane0_occupied and lane1_occupied:
            print "lanes occupied"
            self.lane = -1

    def lane_occupied(self, lane, points):
        num_obstacles = 0
        pos = np.array([self.pose.pose.pose.position.x, self.pose.pose.pose.position.y])

        _, min_param = lane.closest_point(pos)
        max_param = min_param + 1.0

        for p in points.T:
            p = p.flatten()
            closest, param = lane.closest_point(p, 0.1, min_param, max_param)
            distance = np.sqrt(np.sum(np.square(closest - p)))

            if distance < 0.15:
                num_obstacles += 1
                if num_obstacles > 10:
                    return True

        return False

    def on_control(self, tmr):

        if self.lane == -1:
            speed_msg = SpeedCommand()
            speed_msg.value = 0.0
            self.speed_pub.publish(speed_msg)
            return

        lane = self.map.lanes[self.lane]
        pos = np.array([self.pose.pose.pose.position.x, self.pose.pose.pose.position.y])
        lookahead_point, param = lane.lookahead_point(pos, 0.7)

        vec = lookahead_point[0] - pos
        desired_angle = math.atan2(vec[1], vec[0])

        speed_msg = SpeedCommand()
        speed_msg.header.frame_id = "base_link"
        speed_msg.header.stamp = rospy.Time.now()
        speed_msg.value = 0.3 + 0.4 * (1.0 - lane.curvature(param))
        self.speed_pub.publish(speed_msg)

        steering_msg = SteeringCommand()
        steering_msg.value = desired_angle
        steering_msg.header.frame_id = "base_link"
        steering_msg.header.stamp = rospy.Time.now()
        self.steering_pub.publish(steering_msg)

        viz_msg = Marker(type=Marker.SPHERE, action=Marker.ADD)
        viz_msg.header.frame_id = "map"
        viz_msg.scale.x = 0.1
        viz_msg.scale.y = 0.1
        viz_msg.scale.z = 0.1
        viz_msg.color.b = 1.0
        viz_msg.color.a = 1.0
        viz_msg.id = 1
        viz_msg.color.b = 0.0
        viz_msg.color.g = 1.0
        viz_msg.pose.position.x = lookahead_point[0][0]
        viz_msg.pose.position.y = lookahead_point[0][1]
        self.lookahead_pub.publish(viz_msg)


    def on_shutdown(self):
        speed_msg = SpeedCommand()
        speed_msg.value = 0.0
        self.speed_pub.publish(speed_msg)


if __name__ == "__main__":
    Navigation()
