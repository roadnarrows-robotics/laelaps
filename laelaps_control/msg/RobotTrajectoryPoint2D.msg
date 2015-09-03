# Laelaps laelaps_control/RobotTrajectoryPoint2D
#
# Robot 2D trajectory point.
#
# A 2D trajectory point defines position, dynamics, and timing for the robot
# to execute.
#

geometry_msgs/Pose2D  pose            # 2D pose [meters, meters, radians].
float64               velocity        # Velocity at pose [meters/s].
float64               acceleration    # Acceleration [m/s^2] (not used in v1.0).
float64               effort          # Effort [Watts_e] (not used in v1.0).
duration              time_from_start # Start time to move to this point.
