#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String
from smbus2 import SMBus

class MyPublisherNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(MyPublisherNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # construct publisher
        self.pub = rospy.Publisher('chatter', String, queue_size=10)

    def run(self):
        # publish message every 1 second
        rate = rospy.Rate(1) # 1Hz
        while not rospy.is_shutdown():

            SMBus(12).pec = 1  # Enable PEC
            lanereader_value = SMBus(12).read_byte_data(0x3e, 0x11)

            rospy.loginfo(f"Lanereader: '{lanereader_value}'")
            self.pub.publish(lanereader_value)
            rate.sleep()

if __name__ == '__main__':
    # create the node
    node = MyPublisherNode(node_name='db_laneollowing_sparkfun')
    # run node
    node.run()
    # keep spinning
    rospy.spin()
