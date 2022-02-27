#!/usr/bin/env python3

import rospy
import os
import numpy as np
from geometry_msgs.msg import PoseStamped
from pure_pursuit import *
from std_msgs.msg import Float64
from path_helpers import *


class Navigation:
    """
    This does navigation
    """

    def __init__(self, path: List[Point], look_ahead_distance: float, look_forward_gain: float, wheel_base: float):
        self.steering_pub = rospy.Publisher('steering', Float64, queue_size=10)
        self.path = path
        self.target_index = 0
        self.navigator = PurePursuit(look_ahead_distance, look_forward_gain, path)
        self.car_state = CarState(wheel_base)
        self.previous_point = 0
        self.car_dist = []

    def listener(self):
        rospy.init_node('navigation', anonymous=True)
        rospy.Subscriber('airsimPose', PoseStamped, self.handle_gps_data)
        rospy.Subscriber("sensor/speed", Float64, self.handle_speed_data)
        rate = rospy.Rate(30)
        # Once to get initial starting index
        self.target_index = self.navigator.search_target_index(self.car_state)[0]

        while not rospy.is_shutdown():
            if len(self.path) - 1 > self.target_index:
                steering_angle = self.get_steering_angle()
                self.steering_pub.publish(steering_angle)
            else:
                rospy.loginfo('Done route')
                with open(os.path.abspath(os.path.dirname(__file__)) + '/nav_metric.txt', 'w') as metrics_file:
                    metrics_file.write(f"Average distance to the optimal path is {np.average(self.car_dist)}\n")
                    metrics_file.write(f"Median distance to the optimal path is {np.median(self.car_dist)}\n")
                    metrics_file.write(f"Max distance along the entire trip is {np.max(self.car_dist)}")
                    rospy.loginfo('wrote')
                # rospy.sigal_shutdown("Done route")
            rate.sleep()

    def handle_gps_data(self, position: PoseStamped):
        curr_point = Point((position.pose.position.x, position.pose.position.y))
        quaternion = (position.pose.orientation.x, position.pose.orientation.y,
                      position.pose.orientation.z, position.pose.orientation.w)
        self.car_state.update_pos(curr_point, quaternion)

    def handle_speed_data(self, speed: Float64):
        self.car_state.update_speed(speed.data)

    """
    Returns a steering angle for the car in Airsim range of -1 to 1
    """

    def get_steering_angle(self) -> float:
        ackerman_angle_rad, new_target_index = self.navigator.pure_pursuit_steer_control(self.car_state)

        # If a new goal point is selected set the previous point to the last point
        if new_target_index != self.target_index:
            self.previous_point = self.target_index

        self.target_index = new_target_index
        ackerman_angle_ratio = math.degrees(ackerman_angle_rad) / 45
        rospy.loginfo(
            f"""Target point is {path[self.target_index].coordinate[0]}, {path[self.target_index].coordinate[1]},
                      car position {self.car_state.point.coordinate[0]}, {self.car_state.point.coordinate[1]}""")
        rospy.loginfo(f'Current steering angle in degrees {math.degrees(ackerman_angle_rad)}')

        # Calc the distance of that car to the points
        if self.target_index != self.previous_point:
            cur_pos = (self.car_state.point.coordinate[0], self.car_state.point.coordinate[1])
            previous_cord = (path[self.target_index].coordinate[0], path[self.target_index].coordinate[1])
            target_cord = (path[self.previous_point].coordinate[0], path[self.previous_point].coordinate[1])

            distance = np.abs(
                np.cross(np.subtract(target_cord, previous_cord), np.subtract(previous_cord, cur_pos))) \
                       / np.linalg.norm(np.subtract(target_cord, previous_cord))
            rospy.loginfo(f'Distance {distance}')
            self.car_dist.append(distance)

        return ackerman_angle_ratio


if __name__ == "__main__":
    path = read_points(os.path.abspath(os.path.dirname(__file__)) + "/coords.txt")
    navigation = Navigation(look_ahead_distance=4.0, look_forward_gain=0.1, path=path, wheel_base=2.2)
    navigation.listener()
