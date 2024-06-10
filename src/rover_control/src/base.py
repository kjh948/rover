#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, Float32
from sensor_msgs.msg import Imu

import serial
import struct
import json
import time
from robot import Robot

import subprocess

pub_temperature = rospy.Publisher('/cpu_temperature', Float32, queue_size=10)
pub_imu = rospy.Publisher('/imu_raw', Imu, queue_size=10)
pub_battery = rospy.Publisher('/battery', Float32, queue_size=10)


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


    # print(rover_conn.ina219_info())
    # print(rover_conn.imu_info())

def callback_battery(event):    
    battery = rover_conn.ina219_info()
    battery = float(json.loads(battery)['bus_V'])
    battery = (battery - 9.75)*100/3
    bat = Float32(battery)
    
    pub_battery.publish(bat)


def callback_temperature(event):    
    t = subprocess.run(['vcgencmd', 'measure_temp'], stdout=subprocess.PIPE).stdout.decode('utf-8').split("=")[1].split("'C")[0]
    temp = Float32(float(t))
    pub_temperature.publish(temp)

def callback_imu(event):
    imu = Imu
    imu = rover_conn.imu_info()
    pass

def node():    

    rospy.init_node('base', anonymous=True)    
    rospy.Subscriber("/cmd_vel", Twist, cmd_callback)

    rate = rospy.Rate(20) # 10hz

    port_name = rospy.get_param('~port','/dev/ttyUSB0')

    global rover_conn
    rover_conn = Robot(port_name)
    rover_conn.connect()
    rospy.loginfo("mcu was initialized and ready now")

    rospy.Timer(rospy.Duration(1), callback_battery)
    rospy.Timer(rospy.Duration(1), callback_temperature)
    # rospy.Timer(rospy.Duration(1/20.), callback_imu)
    rospy.spin()
    # while not rospy.is_shutdown():
    #     battery = rover_conn.ina219_info()
    #     imu = rover_conn.imu_info()
    #     pub_imu.publish(hello_str)
    #     pub_battery.publish(hello_str)
    #     rate.sleep()
    # #    
    
if __name__ == '__main__':

    try:
        node()
    except KeyboardInterrupt:
        pass
