#!/usr/bin/env python3
import time

import rospy
from smbus2 import SMBus
from duckietown_msgs.msg import WheelsCmdStamped, WheelEncoderStamped
from subscribers import Subscribers

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

node_type=NodeType.GENERIC
"""


class AutonomusDuck(Subscribers):

    def __init__(self, node_name):
        super().__init__(node_name=node_name)
        # construct publisher
        self.pub_wheels_cmd = rospy.Publisher("/blubot/wheels_driver_node/wheels_cmd", WheelsCmdStamped, queue_size=10)
        self.msg_wheels_cmd = WheelsCmdStamped()
        self.arr = SMBus(14)

    def is_obstacle(self):
        # print("distance: ", self.tof_data)
        if 0.1 < self.tof_data < 0.25:
            return True
        return False

    def show_stats(self):
        print(f"Range: {round(self.tof_data, 2)}\n"
              f"Left motor: {self.left_wheel}\n"
              f"Right motor: {self.right_wheel}\n"
              f"Is Obstacle: {self.is_obstacle()}\n"
              f"----------------------")

    def shutdown(self):
        self.msg_wheels_cmd.vel_right = 0
        self.msg_wheels_cmd.vel_left = 0
        self.pub_wheels_cmd.publish(self.msg_wheels_cmd)


    def run(self):
        rate = rospy.Rate(20)
        last_dir = None
        array_value = self.arr.read_byte_data(0x3e, 0x11)

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
    node = AutonomusDuck(node_name="AutonomusDuck")
    # run node
    while True:
        try:
            node.show_stats()
            if node.is_obstacle():
                node.shutdown()
            else:
                node.run()
        except ValueError:
            pass



