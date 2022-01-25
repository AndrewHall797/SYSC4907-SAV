#!/usr/bin/env python3
import math
import time

import rospy
from mapping_navigation.msg import PathData
from sensor_msgs.msg import PointCloud
from std_msgs.msg import Float64
import os

class CruiseControl:

    def __init__(self):
        self.steeringPub = rospy.Publisher("steering", Float64, queue_size = 10)
        self.brakingPub = rospy.Publisher("braking", Float64, queue_size = 10)
        self.throttlePub = rospy.Publisher("throttling", Float64, queue_size = 10)

        # This depends on exp_factor in pidToCarValues()
        self.speedAdjustmentFactor = 1.25
        self.currentSpeed = 0.0
        self.targetSpeed = 5.0 * self.speedAdjustmentFactor

        self.speed_difference_sum = 0
        self.lastError = 0
        self.speedDifference = 0

        # Used for automated testing
        pidCoefficientsFile = open(os.path.abspath(os.path.dirname(__file__)) + "/pid_values.txt")
        self.outputSpeedFile = open(os.path.abspath(os.path.dirname(__file__)) + "/speed_output.txt", "w")

        self.kP = float(pidCoefficientsFile.readline())

        # Used in integral windup mitigation. See publish_results()
        self.potentialKiValue = float(pidCoefficientsFile.readline())
        self.clampValue = 0.995  # Must be in range of [0, 1)

        self.kI = self.potentialKiValue

        self.kD = float(pidCoefficientsFile.readline())

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

    # Converts the PID controller output value in the range of [0, 1) that the throttle and brake can have
    def pidToCarValues(self, inp):
        # A lower value means a more gradual throttling and acceleration. However a higher input is needed to get a
        # output value that is needed for a car to achieve its target speed than the PID controller will output. To deal
        # with this, make the target speed higher than what is desired internally using self.speedAdjustmentFactor
        exp_factor = -0.5
        return max(1 - math.exp(exp_factor * inp), 0.0)

    def publish_results(self):

        delta_time = time.time() - self.lastTime
        self.lastTime += delta_time

        # Find the inputs to the PID controller: value for proportional, integral and differential

        self.speedDifference = self.targetSpeed - self.currentSpeed
        proportional_input = self.speedDifference * self.kP

        self.speed_difference_sum = (self.speed_difference_sum + self.speedDifference) * delta_time
        integral_input = self.speed_difference_sum * self.kI

        differential_input = self.kD * ((self.speedDifference - self.lastError) / delta_time)
        self.lastError = self.speedDifference

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

        # Need to stop integral windup. If PID controller is saying to brake or accelerate but car isn't for whatever reason
        # then the integral will increase in magnitude continously. Then when the car is free to move, the pid output will stay near max
        # throttle or break even as it nears target speed as the integral component will take a long time to reach an appropriate
        # brake or throttle value

        clamping_has_effect = translated_value > self.clampValue
        same_signs = (pid_controller_output > 0.0 and self.speedDifference > 0.0) or (pid_controller_output < 0.0 and self.speedDifference < 0.0)

        # Integral windup occurs when car is told to accelerate when is it below the target speed; hence the same_signs variable.
        # Same logic but in reverse when it comes to speed for braking
        if clamping_has_effect and same_signs:

            # Integral windup may be detected, but if kI is still small let it continue changing. For example, if kI is
            # 0.05 and integral windup is detected but kI is now at 0.25, jumping directly to the potentiaKiValue can
            # result in an abrupt change of kI
            if abs(self.kI) > self.potentialKiValue:
                if pid_controller_output < 0.0:
                    self.kI = -self.potentialKiValue
                else:
                    self.kI = self.potentialKiValue

        self.outputSpeedFile.write("{},{}\n".format(delta_time, self.currentSpeed))

if __name__ == "__main__":
    # Do something
    cc = CruiseControl()
    cc.listener()
