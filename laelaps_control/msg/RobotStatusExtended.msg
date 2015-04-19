### Laelaps laelaps_control/RobotStatusExtended message

# Base industrial_msgs/RobotStatus
Header    header    # standard header

industrial_msgs/RobotMode mode              # robot operation mode: AUTO MANUAL
industrial_msgs/TriState  e_stopped         # emergency stopped: TRUE FALSE
industrial_msgs/TriState  drives_powered    # servos powered: TRUE FALSE
industrial_msgs/TriState  motion_possible   # motion possbile: TRUE FALSE
industrial_msgs/TriState  in_motion         # in motion: TRUE FALSE
industrial_msgs/TriState  in_error          # in error condition: TRUE FALSE
int32 error_code                            # Laelaps error code

# Laelaps extensions
MotorCtlrHealth[] motor__ctlr_health        # motor controller health
MotorHealth[]     motor_health              # motor health
