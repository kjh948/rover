#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, Float32
import serial
import struct
import json
import time
from robot import Robot

def cmd_callback(msg):
    """Extract linear.x and angular.z from Twist msg."""
    cmd_vel_x = msg.linear.x  # meters/sec
    cmd_vel_z = msg.angular.z  # radians/sec

    rospy.loginfo("linear="+str(cmd_vel_x)+"    angular="+str(cmd_vel_z))

    #range of linear vel = 0.1 < vel < 1.0
    min_vel = 0.1
    max_vel = 1.0
    
    angular_scale = 2.0
    l = cmd_vel_x - angular_scale * cmd_vel_z
    r = cmd_vel_x + angular_scale * cmd_vel_z

    rospy.loginfo("pwm l="+str(l)+"  "+"pwm r="+str(r))

    global rover_conn
    rover_conn.speed_input(left_speed=l*100, right_speed=r*100)


    #ina219_info


def control():    
    rospy.init_node('base', anonymous=True)    
    rospy.Subscriber("/cmd_vel", Twist, cmd_callback)

    port_name = rospy.get_param('~port','/dev/ttyUSB0')

    global rover_conn
    rover_conn = Robot(port_name)
    rover_conn.connect()
    rospy.loginfo("mcu was initialized and ready now")


    rospy.spin()    
    
if __name__ == '__main__':

    try:
        control()
    except KeyboardInterrupt:
        pass
