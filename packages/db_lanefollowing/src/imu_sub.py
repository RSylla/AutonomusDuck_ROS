#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu

list_linear_x = []
list_linear_y = []
list_linear_z = []
list_angular_x = []
list_angular_y = []
list_angular_z = []

def callback(data):
    list_linear_x.append(data.linear_acceleration.x)
    list_linear_y.append(data.linear_acceleration.y)
    list_linear_z.append(data.linear_acceleration.z)
    list_angular_x.append(data.angular_velocity.x)
    list_angular_y.append(data.angular_velocity.y)
    list_angular_z.append(data.angular_velocity.z)
    avg_linear_x = sum(list_linear_x) / len(list_linear_x)
    avg_linear_y = sum(list_linear_y) / len(list_linear_y)
    avg_linear_z = sum(list_linear_z) / len(list_linear_z)
    avg_angular_x = sum(list_angular_x) / len(list_angular_x)
    avg_angular_y = sum(list_angular_y) / len(list_angular_y)
    avg_angular_z = sum(list_angular_z) / len(list_angular_z)
    print(f"Average angular velocity: ({avg_angular_x}, {avg_angular_y}, {avg_angular_z})")
    print(f"Average linear acceleration: ({avg_linear_x}, {avg_linear_y}, {avg_linear_z})")
    print(f"length of list: {len(list_linear_x)}")
    
def listener():
    rospy.init_node('imu_sub.py', anonymous=True)
    rospy.Subscriber('/blubot/imu_node/imu_data', Imu, callback)
    
    
    rospy.spin()
if __name__ == '__main__':
    listener()
