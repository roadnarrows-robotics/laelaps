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
BaseState     base
WheelState[]  wheel

float64   odometer    # robot odometer m (meters)
float64   velocity    # robot linear velocity v (meters/second)
float64[] proximity   # proximity sensors (meters)

string  name          # motor name
float64 odometer      # wheel odometer m (meters)
int64   encoder       # motor encoder position (pulses)
float64 velocity      # wheel linear velocity v (meters/second)
int32   speed         # motor raw speed (pulses/second)
float64 power_elec    # motor input electrical power Pe (watts)
