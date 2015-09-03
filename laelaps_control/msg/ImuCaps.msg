### Laelaps laelaps_control/ImuCaps
#
# Robot base Inertia Measurement Unit capabilities and specifications.
#

string              name      # Sensor name. [base_imu].
                              #   There is 1 built-in smart IMU.
uint8               total_dof # Total number of IMU degrees of freedom.
float64             max_hz    # Maximum sense/read rate [Hertz]. 

# Accelerometer Sensor
uint8               accel_dof # Accelerometer number of orthogonal DoFs.
                              #   Set to 0 if no accelerometer.
float64             accel_min # Minimum detectable acceleration [meters/s^2].
float64             accel_max # Maximum detectable acceleration [meters/s^2].
float64             accel_res # Accelerometer resolution [meters/s^2].
geometry_msgs/Point accel_off # Location offset from center of robot [meters].

# Gyroscope Sensor
uint8               gyro_dof  # Gyroscope number of orthogonal DoFs.
                              #   Set to 0 if no gyroscope.
float64             gyro_min  # Minimum detectable rotation [radians/s].
float64             gyro_max  # Maximum detectable rotation [radians/s].
float64             gyro_res  # Gyroscope resolution [radians/s].
geometry_msgs/Point gyro_off  # Location offset from center of robot [meters].

# Magnetometer Sensor
uint8               mag_dof   # Magnatometer number of orthogonal DoFs.
                              #   Set to 0 if no magnetometer.
float64             mag_min   # Minimum detectable magnetic field [Tesla].
float64             mag_max   # Maximum detectable magnetic field [Tesla].
float64             mag_res   # Magnetometer resolution [Tesla].
geometry_msgs/Point mag_off   # Location offset from center of robot [meters].
