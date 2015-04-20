### Laelaps laelaps_control/Dynamics message
#
# Robot dynamics state.
# 
Header                header          # standard header
geometry_msgs/Pose2D  robot_pose      # robot 2D pose (meters, meters, radians)
float64               robot_velocity  # robot linear velocity v (meters/second)

# wheels and motors
string[]  name                    # motor-wheel assembly name
float64[] wheel_odometer          # wheel odometer m (meters)
float64[] wheel_linear_velocity   # wheel linear velocity v (meters/second)
float64[] wheel_angular_velocity  # wheel angular velocity w (radians/second)
int64[]   motor_encoder           # motor encoder position (pulses)
int32[]   motor_speed             # motor raw speed (pulses/second)
float32[] motor_power_elec        # motor input electrical power Pe (watts)
