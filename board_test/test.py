from wave_rover_serial import Robot

# Initialize the Robot with the appropriate serial port and baud rate
robot = Robot('/dev/ttyUSB0')

# Connect to the robot
robot.connect()

# Send a command to set motor speeds
robot.speed_input(left_speed=100, right_speed=100)

# Send a command to get the IMU information and read the response
data = robot.imu_info()
print(data)

# Safely disconnect when done
robot.disconnect()
