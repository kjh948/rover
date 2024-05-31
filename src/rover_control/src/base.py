#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, Float32
import serial
import struct
import json
import time
ROVER_SERIAL_PORT = '/dev/ttyS2'
ROVER_BAUD_RATE = 1000000

COMMANDS = {
    "MOVE": {"T": 1, "L": 0, "R": 0},  # Normal forward, adjust speed as needed
    "STOP": {"T": 0}  # Emergency stop
}

def open_serial_connection(port, baud_rate):
    """Attempts to open a serial connection to a given port with specified baud rate."""
    try:
        return serial.Serial(port, baud_rate, timeout=1)
    except serial.SerialException as e:
        print(f"Failed to open serial port {port}: {e}")
        return None

def send_command_to_rover(serial_conn, command):
    """Sends a JSON-formatted command to the rover over a serial connection."""
    command_str = json.dumps(command) + '\n'
    try:
        serial_conn.write(command_str.encode())
    except serial.SerialException as e:
        print(f"Failed to send command to rover: {e}")

def cmd_callback(msg):
    """Extract linear.x and angular.z from Twist msg."""
    cmd_vel_x = msg.linear.x  # meters/sec
    cmd_vel_z = msg.angular.z  # radians/sec

    #range of linear vel = 0.1 < vel < 1.0
    min_vel = 0.1
    max_vel = 1.0
    
    angular_scale = 1.0
    l = cmd_vel_x - angular_scale * cmd_vel_z
    r = cmd_vel_x + angular_scale * cmd_vel_z
    COMMANDS['MOVE']['L'] = l
    COMMANDS['MOVE']['R'] = r

    send_command_to_rover(rover_conn, COMMANDS)
    

rover_conn = open_serial_connection(ROVER_SERIAL_PORT, ROVER_BAUD_RATE)

def control():    
    rospy.init_node('base', anonymous=True)    
    rospy.Subscriber("/cmd_vel", Twist, cmd_callback)
    rospy.spin()    
    
if __name__ == '__main__':

    try:
        control()
    except KeyboardInterrupt:
        pass