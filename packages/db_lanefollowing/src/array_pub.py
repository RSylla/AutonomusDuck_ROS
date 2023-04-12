#!/usr/bin/env python3

import rospy
import os
from std_msgs.msg import String
from smbus2 import SMBus

def talker():
    vechicle_name = os.environ["VEHICLE_NAME"]
    pub = rospy.Publisher(f'/{vechicle_name}/line_array', String, queue_size=20)
    rospy.init_node('array_pub', anonymous=True)
    
    rate = rospy.Rate(20) # 10hz
    arr = SMBus(1)
    while not rospy.is_shutdown():
        array_value = bin(arr.read_byte_data(0x3e, 0x11))[2:].zfill(8)
        pub.publish(array_value)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass