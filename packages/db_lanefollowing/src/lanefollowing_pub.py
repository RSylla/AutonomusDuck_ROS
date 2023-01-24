#!/usr/bin/env python3

import rospy
from duckietown.dtros import DTROS, NodeType, TopicType
from std_msgs.msg import String
from smbus2 import SMBus
from duckietown_msgs.msg import WheelsCmdStamped
from time import sleep
"""
Brake the right wheel:
Little left if arr <= 6
More left if arr <= 3
Hard left if arr <= 1

Brake the left wheel:
Little right if arr >= 32
More right if arr >= 64
Hard hard if arr >= 128

backwards if arr == 0

forwards if arr in range 48, 24, 12, 16, 28,56, 8
"""

class MyPublisherNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(MyPublisherNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # construct publisher
        self.pub_wheels_cmd = rospy.Publisher("/blubot/wheels_driver_node/wheels_cmd", WheelsCmdStamped, queue_size=10)
        self.msg_wheels_cmd = WheelsCmdStamped()

    def read_lane(self):
        while not rospy.is_shutdown():
            SMBus(1).pec = 1  # Enable PEC

            array_value = SMBus(1).read_byte_data(0x3e, 0x11)
            print(array_value)


    def shutdown(self):
        self.msg_wheels_cmd.vel_right = 0
        self.msg_wheels_cmd.vel_left = 0
        self.pub_wheels_cmd.publish(self.msg_wheels_cmd)

    def motor_driver(self):
        rate = rospy.Rate(20)

        last_dir = None
        while not rospy.is_shutdown():
            SMBus(1).pec = 1  # Enable PEC

            array_value = SMBus(1).read_byte_data(0x3e, 0x11)
            # print(array_value)


            if array_value == 255:
                self.msg_wheels_cmd.vel_right = 0
                self.msg_wheels_cmd.vel_left = 0
                self.pub_wheels_cmd.publish(self.msg_wheels_cmd)

            else:
                if array_value == 0:
                    if last_dir == 1:
                        self.msg_wheels_cmd.vel_right = 0
                        self.msg_wheels_cmd.vel_left = 0.2
                        self.pub_wheels_cmd.publish(self.msg_wheels_cmd)
                    elif last_dir == 0:
                        self.msg_wheels_cmd.vel_right = 0.2
                        self.msg_wheels_cmd.vel_left = 0
                        self.pub_wheels_cmd.publish(self.msg_wheels_cmd)

                # Masin keerab j2rsult paremale
                elif array_value == 1:
                    self.msg_wheels_cmd.vel_right = 0.2
                    self.msg_wheels_cmd.vel_left = 0.6
                    self.pub_wheels_cmd.publish(self.msg_wheels_cmd)
                    last_dir = 1

                # Masin keerab j2rsult vasakule
                elif array_value >= 128:
                    self.msg_wheels_cmd.vel_right = 0.6
                    self.msg_wheels_cmd.vel_left = 0.2
                    self.pub_wheels_cmd.publish(self.msg_wheels_cmd)
                    last_dir = 0

                # Masin keerab paremale
                elif 1 < array_value <= 3:
                    self.msg_wheels_cmd.vel_right = 0.2
                    self.msg_wheels_cmd.vel_left = 0.5
                    self.pub_wheels_cmd.publish(self.msg_wheels_cmd)
                    last_dir = 1

                # Masin keerab vasakule
                elif 33 <= array_value <= 64 and array_value != 48:
                    self.msg_wheels_cmd.vel_right = 0.5
                    self.msg_wheels_cmd.vel_left = 0.2
                    self.pub_wheels_cmd.publish(self.msg_wheels_cmd)
                    last_dir = 0

                # Masin veidikene paremale
                elif 4 <= array_value >= 7:
                    self.msg_wheels_cmd.vel_right = 0.3
                    self.msg_wheels_cmd.vel_left = 0.4
                    self.pub_wheels_cmd.publish(self.msg_wheels_cmd)
                    last_dir = 1

                # Masin keerab veidikene vasakule
                elif 24 <= array_value <= 32:
                    self.msg_wheels_cmd.vel_right = 0.4
                    self.msg_wheels_cmd.vel_left = 0.3
                    self.pub_wheels_cmd.publish(self.msg_wheels_cmd)
                    last_dir = 0

                # Masin s6idab otse t2iskiirusel
                else:
                    self.msg_wheels_cmd.vel_right = 0.8
                    self.msg_wheels_cmd.vel_left = 0.8
                    self.pub_wheels_cmd.publish(self.msg_wheels_cmd)

            rate.sleep()



if __name__ == '__main__':
    # create the node
    node = MyPublisherNode(node_name='lanefollowing_pub')
    # run node
    try:
        node.motor_driver()
        # node.read_lane()
    except OSError:
        node.shutdown()
