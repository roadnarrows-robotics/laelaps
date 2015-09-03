### Laelaps laelaps_control/IlluminanceState message
#
# The state of all of the illuminance sensors.
# 

# sensors state
Header    header      # Standard header.
string[]  name        # Sensor name.
float64[] illuminance # Measured illuminance [lux].
                      #   Note: Values outside of the [min, max] should be
                      #         be discarded.
