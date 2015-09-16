### Laelaps laelaps_control/Velocity
#
# Powertrain velocities.
#
# Set powertrain target velocities. Any drives not specified remain at their
# respective current velocities. Angular velocity is measured at the fixed
# rotational drive_shaft - wheel - tire powertrain components.
#

# powertrain velocities
Header    header      # Standard header.
string[]  names       # Powertrain names: [left_front, right_front,
                      #                   left_rear, right_rear].
float64[] velocities  # Angular drive shaft/wheel velocities [radians/seconds]
