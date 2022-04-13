#!/usr/bin/env python3
import time

from pid_controller import PIDController

import rospy
from mapping_navigation.msg import PathData
from std_msgs.msg import Float64


class CruiseControl:

    def __init__(self):
        self.steeringPub = rospy.Publisher("steering", Float64, queue_size = 10)
        self.brakingPub = rospy.Publisher("braking", Float64, queue_size = 10)
        self.throttlePub = rospy.Publisher("throttling", Float64, queue_size = 10)

        self.pidController = PIDController()
        self.currentSpeed = 0.0
        self.lastTime = time.time()

    def listener(self):
        rospy.init_node("cruise_control", anonymous=True)
        # Define Path data structure
        rospy.Subscriber("pathData", PathData, self.handle_path_data)
        rospy.Subscriber("sensor/speed", Float64, self.handle_speed_data)
        rospy.Subscriber("target_speed", Float64, self.handle_new_target_speed)

        # Node is publisher and subscriber- cannot use spin; the publisher methods will never get called

        # Probably should have same rate for this node and speedometer to make them more in sync
        # Higher the rate the smoother the constant speed is, but will take more CPU power
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            self.publish_results()
            rate.sleep()

    def handle_path_data(self, data):
        print("Obtained path data")

    def handle_new_target_speed(self, new_target_speed: Float64):
        self.pidController.update_target_speed(new_target_speed.data)

    def handle_speed_data(self, speed: Float64):
        self.currentSpeed = speed.data

    def publish_results(self):

        delta_time = time.time() - self.lastTime

        if delta_time <= 0.0:
            return

        self.lastTime += delta_time
        throttle_value = self.pidController.update_pid_output(self.currentSpeed, delta_time)
        self.throttlePub.publish(throttle_value)
        rospy.loginfo("Setting throttle: {}".format(throttle_value))


if __name__ == "__main__":
    # Do something
    cc = CruiseControl()
    cc.listener()
