#!/usr/bin/env python3
import rospy
import os
from smbus2 import SMBus
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped, WheelEncoderStamped
from sensor_msgs.msg import Range
import time


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
        # Create i2c bus instance
        self.arr = SMBus(1)

        # initial values to be overwritten
        self.tof_data = 0
        self.right_wheel = 0
        self.left_wheel = 0

        # construct wheel encoders and tof sensor subscribers
        self.left_encoder_data = rospy.Subscriber(f'/{self.veh_name}/left_wheel_encoder_node/tick',
                                                    WheelEncoderStamped, self.left_callback)
        self.right_encoder_data = rospy.Subscriber(f'/{self.veh_name}/right_wheel_encoder_node/tick',
                                                    WheelEncoderStamped, self.right_callback)
        self.tof_sensor = rospy.Subscriber(f'/{self.veh_name}/front_center_tof_driver_node/range',
                                                    Range, self.tof_callback)

    def left_callback(self, data):
        self.left_wheel = data.data

    def right_callback(self, data):
        self.right_wheel = data.data

    def tof_callback(self, data):
        self.tof_data = data.range

    def is_obstacle(self):
        if 0.1 < self.tof_data < 0.25:
            return True
        return False

    def shutdown(self):
        self.msg_wheels_cmd.vel_right = 0
        self.msg_wheels_cmd.vel_left = 0
        self.pub_wheels_cmd.publish(self.msg_wheels_cmd)


    def arr_input(self):
        array_value = bin(self.arr.read_byte_data(0x3e, 0x11))[2:].zfill(8)
        new_values_list = [-5, -3, -2, -1, 1, 2, 3, 5]
        bitsum = 0
        counter = 0

        for index in range(len(array_value)):
            if array_value[index] == "1":
                bitsum += new_values_list[index]
                counter += 1

        if counter == 0 or counter == 8:
            raise ValueError
        elif 3 <= counter <= 5:
            result = bitsum
        else:
            result = bitsum / counter

        return result

    def run(self, v_max, pid):

        right_speed = max(0, v_max - pid)
        left_speed = max(0, v_max + pid)

        if right_speed < 0.05:
            left_speed = v_max

        if left_speed < 0.05:
            right_speed = v_max

        self.msg_wheels_cmd.vel_right = right_speed
        self.msg_wheels_cmd.vel_left = left_speed
        self.pub_wheels_cmd.publish(self.msg_wheels_cmd)
        print(f"V max: {v_max}\n"
              f"V left: {left_speed}\n"
              f"V Right: {right_speed}")


def main():
    # create the node
    node = AutonomusDuck(node_name="AutonomusDuck")

    start_time = time.time()
    previous_error = 0
    integral = 0

    # Set initial parameters for duck's devel run (run, Kp, Ki, Kd, v_max).
    rospy.set_param("/rpidv", [1, 0.065, 0.000215, 0.01385, 0.42])
    """
    rosparam set /rpidv "[1, 0.065, 0.000215, 0.01385, 0.42]"
    Best settings so far:
    "[1, 0.065, 0.000215, 0.01385, 0.42]"
    
    last: 
    "[1, 0.063, 0.000015, 0.0138, 0.45]"
    """
    while True:
        # Measure elapsed time
        delta_time = time.time() - start_time

        # Get parameters from ROS
        run, kp, ki, kd, v_max = rospy.get_param("/rpidv")

        # Input is a value from -4 to 4
        try:
            arr_position = node.arr_input()
        except:
            arr_position = previous_error

        # ---- Section for PID controller --------------------------------
        # Since target is 0, the error is actual (arr position).
        target = 0
        error = arr_position - target

        # Proportional:
        p = kp * error
        print(f"Error: {error}\n"
              f"P: {p}")
        # Integral:
        integral += error * delta_time
        i = ki * integral

        # Clamp integral to avoid wind-up
        i = min(max(i, -1), 1)
        if error == 0:
            i = 0
        print(f"I: {i}")
        # Derivative:
        d = kd * ((error - previous_error) / delta_time)

        pid = min(max(p + i + d, -0.7), 0.7)
        print(f"D: {d}\n"
              f"pid: {pid}")

        # print(f"PID: {pid}")
        # ---- End section ------------------------------------------------

        if run:
            if node.is_obstacle():
                node.shutdown()
            else:
                node.run(v_max, pid)
        else:
            node.shutdown()

        # Overwrite previous error with current error.
        previous_error = arr_position
        start_time = time.time()

        time.sleep(0.02) #to set loops per sec


if __name__ == '__main__':
    main()
