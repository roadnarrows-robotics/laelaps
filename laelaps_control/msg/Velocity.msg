### Laelaps laelaps_control/Velocity
#
# Powertrain velocities.
#
# Set powertrain target velocities. Any drives not specified remain at their
# respective current velocities. Velocity is measured at the tire.
#

# powertrain velocities
Header    header      # standard header
string[]  names       # powertrain names: left_front right_front
                      #                   left_rear right_rear
float64[] velocities  # linear velocities (meters/seconds)
