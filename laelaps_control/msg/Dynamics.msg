### Laelaps laelaps_control/Dynamics message
#
# Robot dynamics state.
# 

# robot dynamics
Header                header          # Standard header.
geometry_msgs/Pose2D  robot_pose      # Robot 2D pose [meters, meters, radians].
float64               robot_odometer  # Robot odometer [meters].
float64               robot_velocity  # Robot linear velocity v [meters/second].

# powertrain dynamics
string[]  name                # Powertrain name: [left_front, right_front,
                              #                  left_rear, right_rear].
float64[] position            # Output shaft position [radians]
float64[] velocity            # Output shaft angular velocity [radians/second]
float32[] torque              # Output torque [N-m]
int64[]   motor_encoder       # Motor encoder position [pulses]
int32[]   motor_speed         # Motor raw speed [pulses/second]
float32[] motor_power_elec    # Motor input electrical power Pe [watts]
