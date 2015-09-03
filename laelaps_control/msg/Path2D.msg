# Laelaps laelaps_control/Path2D
#
# Robot 2D path.
#
# A 2D path defines a set of pose waypoints where each pose specifies the robot
# position x,y and orientation theta. No dynamics (e.g. velocities) are
# specified.
#
# The poses can be specified in some absolute frame of reference (e.g.
# odometer == 0) or relative to the starting/last pose of the robot. 
#
# Example pseudo-path to approach a toy block:
#   path_name     = "toy block"
#   frame_of_ref  = RELATIVE
#   pose_names[0] = "turn left"
#   poses[0]      = (0, 0, radians(90))
#   pose_names[1] = "move thru door"
#   poses[1]      = (1.5, 0, 0)
#   pose_names[2] = "turn right 45"
#   poses[2]      = (0, 0, radians(-45))
#   pose_names[3] = "approach block"
#   poses[3]      = (0.5, 0, 0)
#

# frame of reference enumeration
uint8 ABSOLUTE = 0      # absolute frame of reference
uint8 RELATIVE = 1      # relative frame of reference

Header                  header        # Standard header.
string                  path_name     # Path name.
uint8                   frame_of_ref  # Frame of reference. One of:
                                      #   ABSOLUTE RELATIVE
string[]                pose_names    # Name of poses.
geometry_msgs/Pose2D[]  poses         # 2D poses [meters, meters, radians].
