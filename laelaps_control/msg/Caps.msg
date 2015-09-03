### Laelaps laelaps_control/Caps
#
# Robot base capabilities and specifications.
#

# robotic base capabilities
Dimensions  robot_dim       # Full outside dimensions of the robot.
Dimensions  base_dim        # Dimensions of robot base.
float64     wheelbase       # Distance between centers of front-rear wheels [m].
float64     wheeltrack      # Dist between centerlines of left-right wheels [m].
float64     approach_angle  # Approach (and departure) angle [radians].
                            #   The robot has front-rear symmetry, so the angles
                            #   are identical.
float64     breakover_angle # Breakover (high center) angle Beta [radians].

# built-in features
PowertrainCaps          powertrain    # Powertrain capabilities
ToFSensorCaps           range_sensor  # Time of Flight sensor capabilities
AmbientLightSensorCaps  illumination_sensor # Ambient light sensor capabilities
ImuCaps                 imu           # Inertia Measurement Unit capabilities
