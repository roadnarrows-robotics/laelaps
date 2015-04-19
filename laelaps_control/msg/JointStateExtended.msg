### Laelaps laelaps_control/JointStateExtended message

# Joints in the case of Laelaps are the wheels (4 joints) and the robot base
# "joint" defined at the center of rotation (1 joint).

# Base sensor_msgs/JointState
Header    header        # standard header

string[]  name          # motor name
float64[] position      # robot/wheel position (radians)
float64[] velocity      # robot/wheel velocity w (radians/second)
float64[] effort        # motor torque T = N-m (newton-meters)

# Laelaps extensions
float64[] odometer      # robot/wheel odometer m (meters)
int64[]   encoder       # motor encoder position (pulses)
float64[] velocity_mps  # robot/wheel velocity v (meters/second)
int32[]   speed         # raw speed (pulses/second)
float64[] power_elec    # motor input electrical power Pe (watts)
