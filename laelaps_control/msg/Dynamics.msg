### Laelaps laelaps_control/Dynamics message
#
# Robot dynamics state.
# 

# robot dynamics
Header                header          # Standard header.
float64               robot_odometer  # Robot odometer [meters].
geometry_msgs/Pose2D  robot_pose      # Robot 2D pose [meters, meters, radians].
float64               robot_velocity  # Robot linear velocity v [meters/second].

# powertrain dynamics
string[]  name                    # Powertrain name: [left_front, right_front,
                                  #                  left_rear, right_rear].
float64[] wheel_odometer          # Wheel odometer [meters]
float64[] wheel_linear_velocity   # Wheel linear velocity v [meters/second]
float64[] wheel_angular_velocity  # Wheel angular velocity w [radians/second]
int64[]   motor_encoder           # Motor encoder position [pulses]
int32[]   motor_speed             # Motor raw speed [pulses/second]
float32[] motor_power_elec        # Motor input electrical power Pe [watts]
