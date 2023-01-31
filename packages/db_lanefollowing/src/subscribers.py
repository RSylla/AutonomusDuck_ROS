#!/usr/bin/env python3

import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelEncoderStamped
from sensor_msgs.msg import Range

class Subscribers(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(Subscribers, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # construct publisher
        self.tof_data = 0.0
        self.right_wheel = 0.0
        self.left_wheel = 0.0
        self.left_encoder_data = rospy.Subscriber('/blubot/left_wheel_encoder_node/tick', WheelEncoderStamped, self.leftCallback)
        self.right_encoder_data = rospy.Subscriber('/blubot/right_wheel_encoder_node/tick', WheelEncoderStamped, self.rightCallback)
        self.tof_sensor = rospy.Subscriber('/blubot/front_center_tof_driver_node/range', Range, self.tofCallback)

    def leftCallback(self, data):
        # print(data.data)
        self.left_wheel = data.data

    def rightCallback(self, data):
        # print(data.data)
        self.right_wheel = data.data

    def tofCallback(self, data):
        # print(data.range)
        self.tof_data = data.range
