#!/usr/bin/env python3
import math
import time

import rospy
from mapping_navigation.msg import PathData
from sensor_msgs.msg import PointCloud
from std_msgs.msg import Float64

class CruiseControl:

    def __init__(self):
        self.steeringPub = rospy.Publisher("steering", Float64, queue_size = 10)
        self.brakingPub = rospy.Publisher("braking", Float64, queue_size = 10)
        self.throttlePub = rospy.Publisher("throttling", Float64, queue_size = 10)

        self.currentSpeed: float = 0.0
        self.targetSpeed = 5.0

        self.integral = 0
        self.speedDifference = 0

        # Used in integral windup mitigation. See publish_results()
        self.potentialKiValue = 0.75
        self.clampValue = 0.995  # Must be in range of [0, 1)

        self.kP = 1.0
        self.kI = self.potentialKiValue
        self.kD = 0.05

        self.lastTime = time.time()
        self.translationExpCoefficient = -2.0

    def listener(self):
        rospy.init_node("cruise_control", anonymous=True)
        rospy.Subscriber("lidar", PointCloud, self.handle_lidar_data)
        # Define Path data structure
        rospy.Subscriber("pathData", PathData, self.handle_path_data)
        rospy.Subscriber("sensor/speed", Float64, self.handle_speed_data)

        # Node is publisher and subscriber- cannot use spin; the publisher methods will never get called

        # Probably should have same rate for this node and speedometer to make them more in sync
        # Higher the rate the smoother the constant speed is, but will take more CPU power
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
         self.publish_results()
         rate.sleep()

    def handle_lidar_data(self, data):
        print("Obtained lidar data")

    def handle_path_data(self, data):
        print("Obtained path data")

    def handle_speed_data(self, speed: Float64):
        rospy.loginfo("Handling the speed data")
        self.currentSpeed = speed.data

    # Converts the PID controller value in the range of [0, 1) that the throttle and brake can have
    def pidToCarValues(self, inp):
        return max(1 - math.exp(-2 * inp), 0.0)

    def publish_results(self):

        delta_time = time.time() - self.lastTime
        self.lastTime += delta_time

        # Find the inputs to the PID controller: value for proportional, integral and differential

        self.speedDifference = self.targetSpeed - self.currentSpeed
        proportional_input = self.speedDifference * self.kP

        self.integral = (self.integral + self.speedDifference) * delta_time
        integral_input = self.integral * self.kI

        differential_input = self.kD * (self.speedDifference / delta_time)

        # Compute the value for the throttle or the braking. The output of the PID controller does not correlate to a
        # range of [0, 1] as required for the throttle or brake, hence the call to pidToCarValues

        pid_controller_output = proportional_input + integral_input + differential_input
        translated_value = 0.0

        if pid_controller_output > 0.0:
            translated_value = self.pidToCarValues(pid_controller_output)
            self.throttlePub.publish(translated_value)
            self.brakingPub.publish(0.0)
            rospy.loginfo("Setting throttle: {}".format(translated_value))
        else:
            translated_value = self.pidToCarValues(-pid_controller_output)
            self.brakingPub.publish(translated_value)
            self.throttlePub.publish(0.0)
            rospy.loginfo("Setting brakes: {}".format(translated_value))

        # Need to stop integral windup if car is at correct speed. The throttle or brake value is considered too large
        # when clamping their value has an effect. Translated value is always positive

        clamping_has_effect = translated_value > self.clampValue
        same_signs = pid_controller_output > 0.0 and self.speedDifference > 0.0

        rospy.loginfo("{} | {} | {} | {}".format(proportional_input, integral_input, differential_input, delta_time))

        if clamping_has_effect and same_signs:
            self.kI = 0.0
            rospy.loginfo("Setting kI to 0")
        else:
            self.kI = self.potentialKiValue
            rospy.loginfo("Setting Ki to {}".format(self.potentialKiValue))


if __name__ == "__main__":
    # Do something
    cc = CruiseControl()
    cc.listener()
