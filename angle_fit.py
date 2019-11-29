import circle_fit
import rospy
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import collections
import math

class AngleFit:

    def __init__(self):
        rospy.init_node("circle_fit")
        self.localization_sub = rospy.Subscriber("/sensors/localization/filtered_map", Odometry, self.on_localization,
                                                 queue_size=1)
        self.visualization_pub = rospy.Publisher("fit", MarkerArray, queue_size=10)
        self.rate = rospy.Rate(100)
        self.positions = collections.deque(maxlen=1000)
        self.last_time = rospy.Time.now()
        self.T = 0.17
        self.L = 0.27

        while not rospy.is_shutdown():
            self.rate.sleep()

    def on_localization(self, msg):
        time = rospy.Time.now() - self.last_time
        if (rospy.Time.now() - self.last_time) > rospy.Duration.from_sec(0.2):
            self.last_time = rospy.Time.now()
            self.positions.append([msg.pose.pose.position.x, msg.pose.pose.position.y])

        if len(self.positions) >= 3:
            points = list(self.positions)
            xc, yc, R, var = circle_fit.least_squares_circle(points)
            circ = circle_fit.hyper_fit(points)
            print (circ)
            print (xc, yc, R, var)

            ackermann_i  = math.atan(self.L / (R + (self.T / 2.0)))
            ackermann_o  = math.atan(self.L / (R - (self.T / 2.0)))
            ackermann_virt = math.atan(self.L / R)

            print (ackermann_i, ackermann_o, ackermann_virt)

            arr = MarkerArray()

            msg = Marker(type=Marker.DELETEALL, action=Marker.DELETEALL)
            msg.id = 0
            arr.markers.append(msg)

            msg = Marker(type=Marker.LINE_STRIP, action=Marker.ADD)
            msg.header.frame_id = "map"
            msg.scale.x = 0.05
            msg.color.b = 0
            msg.color.a = 1.0
            msg.color.r = 1.0
            msg.pose.orientation.x = 0.0
            msg.pose.orientation.y = 0.0
            msg.pose.orientation.z = 0.0
            msg.pose.orientation.w = 1.0
            msg.id = 1

            for p in points:
                msg.points.append(Point(p[0], p[1], 0.0))

            arr.markers.append(msg)

            msg = Marker(type=Marker.LINE_STRIP, action=Marker.ADD)
            msg.header.frame_id = "map"
            msg.scale.x = 0.05
            msg.color.g = 1.0
            msg.color.a = 1.0
            msg.pose.orientation.x = 0.0
            msg.pose.orientation.y = 0.0
            msg.pose.orientation.z = 0.0
            msg.pose.orientation.w = 1.0
            msg.id = 2

            for i in range (0, 361):
                xp = xc + R * math.cos(float(i) * (math.pi / 180.0))
                yp = yc + R * math.sin(float(i) * (math.pi / 180.0))
                msg.points.append(Point(xp, yp, 0.0))

            arr.markers.append(msg)
            self.visualization_pub.publish(arr)




if __name__ == "__main__":
    AngleFit()
