# Laelaps laelaps_control/RobotTrajectory2D
#
# Robot 2D trajectory is as set of trajectory 2D waypoints specifying positions,
# dynamics, and timing information.
#
# The pose position can be specified in some absolute frame of reference
# (e.g. robot odometer == 0) or relative to the starting/last pose of the robot.
#

uint8 ABSOLUTE = 0      # absolute frame of reference
uint8 RELATIVE = 1      # relative frame of reference

Header                    header          # Standard header.
string                    traj_name       # Trajectory name.
uint8                     frame_of_ref    # Frame of reference. One of:
                                          #   ABSOLUTE RELATIVE
string[]                  waypoint_names  # Name of waypoints.
RobotTrajectoryPoint2D[]  points          # Trajectory 2D waypoints.
