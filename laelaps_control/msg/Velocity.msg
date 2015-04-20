### Laelaps laelaps_control/Velocity
#
# Wheel-motor velocities.
#
# Set motor target velocities. Any motors not specified remain at their
# respective current velocities.
#
Header    header      # standard header
string[]  name        # motor-wheel name
float64[] velocity    # wheel linear velocity (meters/seconds)
