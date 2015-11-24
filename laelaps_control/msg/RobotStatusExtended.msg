### Laelaps laelaps_control/RobotStatusExtended message
#
# Extened robot status.
#
# The status includes the ROS industrial information plus extensions specific to
# the robot.
#
Header    header    # Standard header.

# Base industrial_msgs/RobotStatus
industrial_msgs/RobotMode mode              # robot operation mode: AUTO MANUAL
industrial_msgs/TriState  e_stopped         # emergency stopped: TRUE FALSE
industrial_msgs/TriState  drives_powered    # servos powered: TRUE FALSE
industrial_msgs/TriState  motion_possible   # motion possbile: TRUE FALSE
industrial_msgs/TriState  in_motion         # in motion: TRUE FALSE
industrial_msgs/TriState  in_error          # in error condition: TRUE FALSE
int32 error_code                            # Laelaps error code

# Laelaps base extensions
float64                   battery           # estimated battery charge [0%-100%]
industrial_msgs/TriState  is_charging       # battery is charging: TRUE FALSE
float64                   voltage           # battery voltage [V]
float64                   current           # estimated total current draw [A]
float64                   temp              # interior temperature [C]
MotorCtlrHealth[]         motor_ctlr_health # motor controllers health
MotorHealth[]             motor_health      # motors health
