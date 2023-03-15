#!/usr/bin/env python3

import numpy as np
import math
import rospy
import os
from duckietown_msgs.msg import WheelEncoderStamped
from std_msgs.msg import String
import json



class Odometry:

    def __init__(self, max_ticks, wheel_radius, wheel_base):
        self.max_ticks = max_ticks  # N_tot = total number of ticks per revolution
        # Radius "R" of duckiebots wheel is 3,3cm.
        self.wheel_radius = wheel_radius / 100  # Insert value measured by ruler, in *meters*
        # Distance between wheels = baseline_wheel2wheel = 10cm / 100 (in meters)
        self.baseline_wheel2wheel = wheel_base / 100
        self.veh_name = os.environ["VEHICLE_NAME"]

        # This data going to be be recieved by subscribing to ros topic
        self.ticks_left = 0
        self.ticks_right = 0

        # construct wheel encoders and tof sensor subscribers
        self.left_encoder_data = rospy.Subscriber(f'/{self.veh_name}/left_wheel_encoder_node/tick',
                                                  WheelEncoderStamped, self._left_encoder_data)
        self.right_encoder_data = rospy.Subscriber(f'/{self.veh_name}/right_wheel_encoder_node/tick',
                                                   WheelEncoderStamped, self._right_encoder_data)

        self.prev_tick_left = 0
        self.prev_tick_right = 0
        # Initial X, Y and angular positions
        self.prev_x = 0
        self.prev_y = 0
        self.prev_angular_pos = 0
        self.prev_distance = 0

    def _left_encoder_data(self, data):
        self.ticks_left = data.data

    def _right_encoder_data(self, data):
        self.ticks_right = data.data

    def get_ticks(self):
        return self.ticks_left, self.ticks_right

    def overwrite_prev_data(self, current_x, current_y, current_ang, tick_l, tick_r, total_distance):
        self.prev_tick_left = tick_l
        self.prev_tick_right = tick_r
        self.prev_x = current_x
        self.prev_y = current_y
        self.prev_angular_pos = current_ang
        self.prev_distance = total_distance

    def get_alpha(self):
        alpha = (2 * math.pi) / self.max_ticks  # "ALPHA" means rotation per tick in radians.
        return alpha

    def delta_phi(self):
        """
        How much would the wheels rotate with the above tick measurements?
        DELTA means the difference between two values.
        Delta ticks means the difference of current and previous encoder ticks.
        """
        delta_ticks_left = self.ticks_left - self.prev_tick_left  # delta ticks of left wheel
        delta_ticks_right = self.ticks_right - self.prev_tick_right  # delta ticks of right wheel
        """
        Calculate Delta-PHI (wheel rotation in radians)
        Formula: DELTA-PHI = DELTA TICKS * ALPHA
        """
        wheel_rotation_left = delta_ticks_left * self.get_alpha()  # total rotation of left wheel
        wheel_rotation_right = delta_ticks_right * self.get_alpha()  # total rotation of right wheel

        return wheel_rotation_left, wheel_rotation_right

    def pose_estimate(self):
        """
        What is the distance travelled by each wheel?
        Delta-Phi is wheel rotation in radians
        Arc distance formula: ARC = Delta-Phi * Radius
        """
        wheel_rotation_left, wheel_rotation_right = self.delta_phi()
        wheel_distance_left = wheel_rotation_left * self.wheel_radius
        wheel_distance_right = wheel_rotation_right * self.wheel_radius
        """
        How much has the robot travelled?
        Point "A" is absolute distance measured from the center of the robots frame (center of wheelbase).
        Robots distance travelled in robot frame [meters], measured from point A.
        """
        absolute_distance = (wheel_distance_left + wheel_distance_right) / 2

        """
        # Calculate robots angular position "Delta-THETA"
        # Formula: (Right wheels distance - Left wheel distance) / distance between wheels

        # New position in enviroment. Expressed by "X" and "Y" cordinates.
        # Delta-X is the change in X axis.
        # Formula: Delta-X = absolute_distance(distance of point "A") * cosines of angular_pos(delta-theta)

        # Delta-Y is the change in Y axis.
        # Formula: Delta-X = absolute_distance(distance of point "A") * sine of angular_pos(delta-theta)
        """
        delta_angular_pos = (wheel_distance_right - wheel_distance_left) / self.baseline_wheel2wheel  # delta-theta in radians
        delta_x = absolute_distance * np.cos(delta_angular_pos)
        delta_y = absolute_distance * np.sin(delta_angular_pos)

        current_x = round(self.prev_x + delta_x, 2)
        current_y = round(self.prev_y + delta_y, 2)
        current_angular_pos = round(self.prev_angular_pos + delta_angular_pos, 2)
        total_distance = round(self.prev_distance + absolute_distance, 3)


        return current_x, current_y, current_angular_pos, total_distance  # x_curr, y_curr, theta_curr


def talker():
    odometry = Odometry(max_ticks=135, wheel_radius=3.1, wheel_base=10)
    pub = rospy.Publisher('/blubot/odometry', String, queue_size=10)
    rospy.init_node('odometry', anonymous=True)
    rate = rospy.Rate(20)  # 10hz
    # looptime = 0
    while not rospy.is_shutdown():
        x_pos, y_pos, angular_pos, total_distance = odometry.pose_estimate()
        ticks_l, ticks_r = odometry.get_ticks()

        msg = {"x": x_pos,
               "y": y_pos,
               "angular": angular_pos,
               "distance": total_distance}

        pub.publish(json.dumps(msg))
        odometry.overwrite_prev_data(x_pos, y_pos, angular_pos, ticks_l, ticks_r, total_distance)
        # looptime += 1
        rate.sleep()



if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
