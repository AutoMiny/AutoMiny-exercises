import rospy
from nav_msgs.msg import Odometry
from autominy_msgs.msg import SpeedCommand


class SpeedPID:

    def __init__(self):
        rospy.init_node("speed_pid")
        self.speed_pub = rospy.Publisher("/actuators/speed", SpeedCommand, queue_size=10)
        self.localization_sub = rospy.Subscriber("/sensors/localization/filtered_map", Odometry, self.on_localization,
                                                 queue_size=1)
        self.speed_sub = rospy.Subscriber("/control/speed", SpeedCommand, self.on_speed, queue_size=1)

        self.pose = Odometry()
        self.rate = rospy.Rate(100)
        self.timer = rospy.Timer(rospy.Duration.from_sec(0.01), self.on_control)

        self.kp = 0.1
        self.ki = 1.0
        self.kd = 0.0
        self.min_i = -1.0
        self.max_i = 1.0

        self.integral_error = 0.0
        self.last_error = 0.0

        self.recv_time = rospy.Time.now()
        self.desired_speed = None

        rospy.on_shutdown(self.on_shutdown)

        while not rospy.is_shutdown():
            self.rate.sleep()

    def on_localization(self, msg):
        self.pose = msg

    def on_speed(self, msg):
        self.recv_time = rospy.Time.now()
        self.desired_speed = msg.value

    def on_control(self, tmr):

        if tmr.last_duration is None:
            dt = 0.01
        else:
            dt = (tmr.current_expected - tmr.last_expected).to_sec()

        if self.desired_speed is None:
            return

        error = self.desired_speed - self.pose.twist.twist.linear.x

        self.integral_error += error * dt
        self.integral_error = max(self.min_i, self.integral_error)
        self.integral_error = min(self.max_i, self.integral_error)

        derivative_error = (error - self.last_error) / dt
        self.last_error = error

        pid_output = self.kp * error + self.kd * derivative_error + self.ki * self.integral_error

        if (rospy.Time.now() - self.recv_time).to_sec() > 0.2:
            pid_output = 0.0

        speed_msg = SpeedCommand()
        speed_msg.header.frame_id = "base_link"
        speed_msg.header.stamp = rospy.Time.now()
        speed_msg.value = self.desired_speed
        self.speed_pub.publish(speed_msg)

    def on_shutdown(self):
        speed_msg = SpeedCommand()
        speed_msg.value = 0.0
        self.speed_pub.publish(speed_msg)


if __name__ == "__main__":
    SpeedPID()
