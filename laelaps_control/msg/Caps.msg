### Laelaps laelaps_control/Caps
#
# Robot capabilities and specifications.
#

# robotic base capabilities
Dimensions  robot_base  # dimensions of robot base
float64     wheelbase   # distance between centers of front-rear wheels (m)
float64     wheeltrack  # distance between centerlines of left-right wheels (m)

# powertrain capabilities
string[]    name          # powertrain name: front rear
float64[]   motor_max_rot # motor maximum rotation (radians/second)
float65[]   pulses        # number of encoder pulses per radian
float64[]   tire_radius   # wheel tire radius (meters)
float64[]   tire_width    # wheel tire width (meters)
