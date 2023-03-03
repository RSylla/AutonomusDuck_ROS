import numpy as np
import math
import rospy
import os
from duckietown_msgs.msg import WheelEncoderStamped
from std_msgs.msg import String
class Odometry:

    def __init__(self):
        self.max_ticks = 135  # N_tot = total number of ticks per revolution
        # Radius "R" of duckiebots wheel is 3,3cm.
        self.wheel_radius = 3.3 / 100  # Insert value measured by ruler, in *meters*
        # Distance between wheels = baseline_wheel2wheel = 10cm / 100 (for meters)
        self.baseline_wheel2wheel = 10 / 100
        self.veh_name = os.environ["VEHICLE_NAME"]

        #Construct odometry publisher
        self.pub_wheels_cmd = rospy.Publisher(f"/{self.veh_name}/duckiebot_odometry",
                                              String, queue_size=10)
        rospy.init_node("odometry", anonymous=True)
        # This data should be recieved by subscribing to ros topic
        self.ticks_left = 0
        self.ticks_right = 0

        # construct wheel encoders and tof sensor subscribers
        self.left_encoder_data = rospy.Subscriber(f'/{self.veh_name}/left_wheel_encoder_node/tick',
                                                  WheelEncoderStamped, self._left_encoder_data)
        self.right_encoder_data = rospy.Subscriber(f'/{self.veh_name}/right_wheel_encoder_node/tick',
                                                   WheelEncoderStamped, self._right_encoder_data)

        self.prev_tick_left = self.ticks_left
        self.prev_tick_right = self.ticks_right

        #Initial X, Y and angular positions
        self.initial_x = 0
        self.initial_y = 0
        self.initial_angular_pos = 0

    def _left_encoder_data(self, data):
        self.ticks_left = data.data

    def _right_encoder_data(self, data):
        self.ticks_right = data.data

    def alpha(self):
        alpha = (2 * math.pi) / self.max_ticks  # "ALPHA" means rotation per tick in radians.
        return alpha

    def delta_ticks(self):
        """
        How much would the wheels rotate with the above tick measurements?
        DELTA means the difference between two values.
        Delta ticks means the difference of current and previous encoder ticks.
        """
        delta_ticks_left = self.ticks_left - self.prev_tick_left  # delta ticks of left wheel
        delta_ticks_right = self.ticks_right - self.prev_tick_right  # delta ticks of right wheel

        return delta_ticks_left, delta_ticks_right

    def delta_phi(self):
        """
        Calculate Delta-PHI (wheel rotation in radians)
        Formula: DELTA-PHI = DELTA TICKS * ALPHA
        """
        delta_ticks_left, delta_ticks_right = self.delta_ticks()
        wheel_rotation_left = delta_ticks_left * self.alpha()  # total rotation of left wheel
        wheel_rotation_right = delta_ticks_right * self.alpha()  # total rotation of right wheel

        return wheel_rotation_left, wheel_rotation_right

    def each_wheel_distance(self):
        """
        # What is the distance travelled by each wheel?
        # Delta-Phi is wheel rotation in radians
        # Arc distance formula: ARC = Delta-Phi * Radius
        """
        wheel_distance_left = wheel_rotation_left * wheel_radius
        wheel_distance_right = wheel_rotation_right * wheel_radius

        return wheel_distance_left, wheel_distance_right

    def absolute_distance(self):
        """
        How much has the robot travelled?
        Point "A" is absolute distance measeured from the center of the robots frame (center of wheelbase).
        Robots distance travelled in robot frame [meters], measured from point A.
        """
        wheel_distance_left, wheel_distance_right = self.each_wheel_distance()
        absolute_distance = (wheel_distance_left + wheel_distance_right) / 2

        return absolute_distance

    def get_position(self):
        """
        # Calculate robots angular position "Delta-THETA"
        # Formula: (Right wheels distance - Left wheel distance) / distance between wheels

        # New position in enviroment. Expressed by "X" and "Y" cordinates.
        # Delta-X is the change in X axis.
        # Formula: Delta-X = absolute_distance(distance of point "A") * cosines of angular_pos(delta-theta)

        # Delta-Y is the change in Y axis.
        # Formula: Delta-X = absolute_distance(distance of point "A") * sine of angular_pos(delta-theta)
        """
        wheel_distance_left, wheel_distance_right = self.each_wheel_distance()
        angular_pos = (wheel_distance_right - wheel_distance_left) / 10  # delta-theta in radians
        delta_x = self.absolute_distance() * np.cos(angular_pos)
        delta_y = self.absolute_distance() * np.sin(angular_pos)

        return delta_x, delta_y, angular_pos

def talker():
    rate = rospy.Rate(10) # 10hz
    node = Odometry()
    while not rospy.is_shutdown():



        rate.sleep()
#
# if __name__ == '__main__':
#     try:
#         talker()
#     except rospy.ROSInterruptException:
#         pass

#
# max_ticks = 135  # N_tot = total number of ticks per revolution
# pi = math.pi
# alpha = (2 * pi) / max_ticks  # "ALPHA" means rotation per tick in radians.
#
# # Ticks data from the encoders to calculate the difference
# ticks_left = 100  # This data should be recieved by subscribing to ros topic
# prev_tick_left = 0  # This should be overwritten in the end of each loop with the value of ticks_left.
#
# ticks_right = 10  # This data should be recieved by subscribing to ros topic
# prev_tick_right = 0  # This should be overwritten in the end of each loop with the value of ticks_right.
#
# # Initial position of duckiebot
# initial_x = initial_y = 0  # linear position in meters along X and Y axis
# initial_angular_pos = 0  # theta0 angular position in radians
#
# # How much would the wheels rotate with the above tick measurements?
# # DELTA means the difference between two values.
# # Delta ticks means the difference of current and previous encoder ticks.
# delta_ticks_left = ticks_left - prev_tick_left  # delta ticks of left wheel
# delta_ticks_right = ticks_right - prev_tick_right  # delta ticks of right wheel
#
# # Calculate Delta-PHI (wheel rotation)
# # Formula: DELTA-PHI = DELTA TICKS * ALPHA
# wheel_rotation_left = delta_ticks_left * alpha  # total rotation of left wheel
# wheel_rotation_right = delta_ticks_right * alpha  # total rotation of right wheel
#
# print(f"The left wheel rotated: {np.rad2deg(wheel_rotation_left)} degrees")
# print(f"The right wheel rotated: {np.rad2deg(wheel_rotation_right)} degrees")
#
# # Radius "R" of duckiebots wheel is 3,3cm.
# wheel_radius = 3.3 / 100  # Insert value measured by ruler, in *meters*
#
# # What is the distance travelled by each wheel?
# # Delta-Phi is wheel rotation in radians
# # Arc distance formula: ARC = Delta-Phi * Radius
# wheel_distance_left = wheel_rotation_left * wheel_radius
# wheel_distance_right = wheel_rotation_right * wheel_radius
#
# print(f"The left wheel travelled: {wheel_distance_left} meters")
# print(f"The right wheel rotated: {wheel_rotation_right} meters")
#
# # How much has the robot travelled?
# # Point "A" is absolute distance measeured from the center of the robots frame (center of wheelbase).
# # robot distance travelled in robot frame [meters], measured from point A.
# absolute_distance = (wheel_distance_left + wheel_distance_right) / 2
# print(f"The robot has travelled: {absolute_distance} meters")
#
# # Distance between wheels = baseline_wheel2wheel = 10cm / 100 (for meters)
# baseline_wheel2wheel = 10 / 100
#
# # Calculate robots angular position "Delta-THETA"
# # Formula: Right wheels distance - Left wheel distance / distance between wheels
# angular_pos = (wheel_distance_right - wheel_distance_left) / 10  # delta-theta in radians
#
# # New position in enviroment. Expressed by "X" and "Y" scale.
# # Delta-X is the change in X axis.
# # Formula: Delta-X = absolute_distance(distance of point "A") * cosines of angular_pos(delta-theta)
# delta_x = absolute_distance * np.cos(angular_pos)
#
# # Delta-Y is the change in Y axis.
# # Formula: Delta-X = absolute_distance(distance of point "A") * sine of angular_pos(delta-theta)
# delta_y = absolute_distance * np.sin(angular_pos)

#!/usr/bin/env python
# license removed for brevity
