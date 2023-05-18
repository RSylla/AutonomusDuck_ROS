#!/usr/bin/env python3

import rospy
import os
from smbus2 import SMBus
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
        self.tof_data = deque([1,1,1,1,1],5)
        self.right_wheel = 0
        self.left_wheel = 0
        self.odo_values = 0
        self.array_value = 0
        self.left_turn = True
        self.executed_left_turn = True
        # construct wheel encoders and tof sensor subscribers
        self.left_encoder_data = rospy.Subscriber(f'/{self.veh_name}/left_wheel_encoder_node/tick',
                                                    WheelEncoderStamped, self.left_callback)
        self.right_encoder_data = rospy.Subscriber(f'/{self.veh_name}/right_wheel_encoder_node/tick',
                                                    WheelEncoderStamped, self.right_callback)
        self.tof_sensor = rospy.Subscriber(f'/{self.veh_name}/front_center_tof_driver_node/range',
                                                    Range, self.tof_callback)
        self.odo = rospy.Subscriber(f'/{self.veh_name}/odometry', String, self.odo_callback)
        self.array = rospy.Subscriber(f'/{self.veh_name}/line_array', String, self.array_callback)

    def odo_callback(self, data):
        self.odo_values = data
        
    def array_callback(self, data):
        self.array_value = data.data
        
    def left_callback(self, data):
        self.left_wheel = data.data

    def right_callback(self, data):
        self.right_wheel = data.data

    def tof_callback(self, data):
        self.tof_data.append(data.range)

    def shutdown(self):
        self.msg_wheels_cmd.vel_right = 0
        self.msg_wheels_cmd.vel_left = 0
        self.pub_wheels_cmd.publish(self.msg_wheels_cmd)
  
    def run(self, v_max, pid):

        right_speed = max(0, v_max - pid)
        left_speed = max(0, v_max + pid)

        if right_speed < 0.05:
            left_speed = v_max + 0.05

        if left_speed < 0.05:
            right_speed = v_max + 0.05

        self.msg_wheels_cmd.vel_right = right_speed
        self.msg_wheels_cmd.vel_left = left_speed
        self.pub_wheels_cmd.publish(self.msg_wheels_cmd)

    def set_wheels_velocity(self, left, right):
        self.msg_wheels_cmd.vel_left = left
        self.msg_wheels_cmd.vel_right = right
        self.pub_wheels_cmd.publish(self.msg_wheels_cmd)

def main():
    # create the node
    node = AutonomusDuck(node_name="AutonomusDuck")

    start_time = time.time()
    prev_error = 0
    integral = 0
    turn_time = 0
    # Set initial parameters for duck's devel run (run, Kp, Ki, Kd, v_max).
    rospy.set_param("/rpidv", [1, 0.064, 0.0001, 0.02, 0.32])
    """
    rosparam set /rpidv "[1, 0.064, 0.0001, 0.02, 0.29]"
    """
    
    while not rospy.is_shutdown():
        # Measure elapsed time
        delta_time = time.time() - start_time
        # Get parameters from ROS
        run, kp, ki, kd, v_max = rospy.get_param("/rpidv")
        
       
        tof_data = sum(node.tof_data)/len(node.tof_data)

        if node.left_turn == True and not node.executed_left_turn:
            print("left turn = True")
            node.set_wheels_velocity(0.2, 0.28)
            time.sleep(1)
            node.executed_left_turn = True

        try:
            error, node.left_turn = error_calculator(node.array_value)
                
        except:
            error = prev_error
        pid, integral, prev_error = pid_controller(error, integral,
                                                   prev_error, delta_time,
                                                   kp, ki, kd)
        
        if 20 < time.time() - turn_time < 21:
            node.executed_left_turn = True
            node.left_turn = True
            
        if run:
            
            if 0.25 < tof_data < 0.35:
                node.set_wheels_velocity(0.2, 0.0)
                time.sleep(0.735)
                node.set_wheels_velocity(0.15, 0.15)
                time.sleep(2.5)
                node.set_wheels_velocity(0.0, 0.2)
                time.sleep(0.85)
                node.set_wheels_velocity(0.15, 0.2)
                time.sleep(0.5)
                turn_time = time.time()
                node.executed_left_turn = False
                node.left_turn = False
            else:
                node.run(v_max, pid)
        else:
            node.shutdown()
        # Overwrite previous error with current error.
        start_time = time.time()
        time.sleep(0.02) #to set loops per sec

if __name__ == '__main__':
    main()