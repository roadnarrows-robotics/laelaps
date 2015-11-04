### Laelaps laelaps_control/ImuAlt message
#
# Alternative IMU message. ROS standard message interface is sensor_msgs/Imu.
#

Header     header     # Standard header.
float64[3] accel      # x,y,z linear acceleration [m/s^2]
float64[3] gyro       # x,y,z angular velocity [radians/s]
float64[3] rpy        # roll,pitch,yaw [radians]

# Axes indices
uint8 X = 0  # X axis index
uint8 Y = 1  # y axis index
uint8 Z = 2  # z axis index

# RPY indices
uint8 ROLL  = 0   # roll index
uint8 PITCH = 1   # pitch index
uint8 YAW   = 2   # yaw index

uint8 NUM_AXES = 3  # number of axes
