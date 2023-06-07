#!/usr/bin/env python3

import rospy
import os
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped, WheelEncoderStamped
from sensor_msgs.msg import Range
import time
from std_msgs.msg import String
from PIDcontroller import pid_controller, error_calculator
from collections import deque


class AutonomusDuck(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(AutonomusDuck, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # construct wheel command publisher
        self.veh_name = os.environ["VEHICLE_NAME"]
        self.pub_wheels_cmd = rospy.Publisher(f"/{self.veh_name}/wheels_driver_node/wheels_cmd",
                                              WheelsCmdStamped, queue_size=10)
        self.msg_wheels_cmd = WheelsCmdStamped()
        rospy.on_shutdown(self.shutdown)

        # initial values to be overwritten
        self.tof_data = deque([1, 1, 1], 3)  # Kalman filter for TOF sensor
        self.right_wheel = 0
        self.left_wheel = 0
        self.odo_values = 0
        self.array_value = 0
        self.left_turn = True
        self.executed_left_turn = True
        # construct wheel encoders, line reader, tof sensor and odometry subscribers
        self.left_encoder_data = rospy.Subscriber(f'/{self.veh_name}/left_wheel_encoder_node/tick',
                                                  WheelEncoderStamped, self.left_encoder_callback)
        self.right_encoder_data = rospy.Subscriber(f'/{self.veh_name}/right_wheel_encoder_node/tick',
                                                   WheelEncoderStamped, self.right_encoder_callback)
        self.tof_sensor = rospy.Subscriber(f'/{self.veh_name}/front_center_tof_driver_node/range',
                                           Range, self.tof_callback)
        self.odo = rospy.Subscriber(f'/{self.veh_name}/odometry', String, self.odometry_callback)
        self.array = rospy.Subscriber(f'/{self.veh_name}/line_array', String, self.line_reader_callback)

    def odometry_callback(self, data):
        self.odo_values = data

    def line_reader_callback(self, data):
        self.array_value = data.data

    def left_encoder_callback(self, data):
        self.left_wheel = data.data

    def right_encoder_callback(self, data):
        self.right_wheel = data.data

    def tof_callback(self, data):
        self.tof_data.append(data.range)

    def shutdown(self):
        """Called upon shutting duckie down with crtl+C"""
        self.msg_wheels_cmd.vel_right = 0
        self.msg_wheels_cmd.vel_left = 0
        self.pub_wheels_cmd.publish(self.msg_wheels_cmd)

    def run(self, v_max, pid):
        """Function for motor control whilst driving on black line"""
        right_speed = max(0, v_max - pid)  # Limit minimum speed to 0 for each wheel.
        left_speed = max(0, v_max + pid)

        if right_speed < 0.05:  # Control motor speed on >= 90 deg angles (or big errors from line reader).
            left_speed = v_max + 0.05

        if left_speed < 0.05:
            right_speed = v_max + 0.05

        self.msg_wheels_cmd.vel_right = right_speed  # Publish motor commands.
        self.msg_wheels_cmd.vel_left = left_speed
        self.pub_wheels_cmd.publish(self.msg_wheels_cmd)

    def set_wheels_velocity(self, left, right):
        """Assistant function for motor control when avoiding obstacle"""
        self.msg_wheels_cmd.vel_left = left
        self.msg_wheels_cmd.vel_right = right
        self.pub_wheels_cmd.publish(self.msg_wheels_cmd)


def main():
    """Main driving function using sensors data to make duckie lil'bit more autonomus."""
    # create the node
    node = AutonomusDuck(node_name="AutonomusDuck")
    # Establish initial values
    start_time = time.time()
    prev_error = 0
    integral = 0
    turn_time = 0

    # Set initial parameters for duck's PID and obstacle avoidance.
    rospy.set_param("/rpidv", [1, 0.064, 0.0001, 0.02, 0.30])
    rospy.set_param("/ao", [[1.2, 0.42, 0.17], [1.6, 0.185, 0.4], [0.5, 0.4, 0.2]])
    """
    rosparam set /rpidv "[1, 0.064, 0.0001, 0.02, 0.29]"
    rosparam set /ao "[[1.2, 0.42, 0.17], [1.6, 0.185, 0.4], [0.5, 0.4, 0.2]]"
    """

    while not rospy.is_shutdown():
        # Measure elapsed time
        delta_time = time.time() - start_time
        # Get parameters from ROS
        run, kp, ki, kd, v_max = rospy.get_param("/rpidv")

        tof_data = sum(node.tof_data) / len(node.tof_data)  # Overwriting tof sensor variable using kalman filter.

        # Check for possibility for left turn and if it's not already executed.
        if node.left_turn == True and not node.executed_left_turn:
            # If that is the case, going for left turn.
            print("left turn = True")
            node.set_wheels_velocity(0.2, 0.28)
            time.sleep(1)
            node.executed_left_turn = True

        try:
            # Getting error calculated from line reader and if it has found left turn mark.
            error, node.left_turn = error_calculator(node.array_value)
        except:
            # If not getting linereader message from topic or out of track then use last known error.
            error = prev_error
        # Calculating PID values
        pid, integral, prev_error = pid_controller(error, integral,
                                                   prev_error, delta_time,
                                                   kp, ki, kd)
        # If left turn not found within 20 sec. after obstacle turn left turn scanner off
        # To avoid false signals.
        if 20 < time.time() - turn_time < 21:
            node.executed_left_turn = True
            node.left_turn = True

        if run:
            # Receive parameters for obstacle avoidance
            # ft-first turn, st-second turn, tt-third turn parameters.
            ft, st, tt = rospy.get_param("/ao")
            if 0.25 < tof_data < 0.42:
                node.set_wheels_velocity(ft[1], ft[2])
                time.sleep(ft[0])
                node.set_wheels_velocity(st[1], st[2])
                time.sleep(st[0])
                node.set_wheels_velocity(tt[1], tt[2])
                time.sleep(tt[0])
                node.set_wheels_velocity(0.3, 0.3)

                turn_time = time.time()  # Initialise left turn start for left turn time counter.
                # Turn on left turn scanner
                node.executed_left_turn = False
                node.left_turn = False
            node.run(v_max, pid)
        else:
            node.shutdown()
        # Overwrite previous error with current error.
        start_time = time.time()
        time.sleep(0.02)  # to set 30 loops per sec


if __name__ == '__main__':
    main()
