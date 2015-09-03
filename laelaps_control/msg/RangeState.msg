### Laelaps laelaps_control/RangeState message
#
# The state of all of the object detection range sensors.
# 

#
float64 NO_OBJECT = 1000000.0   # a million meters away

# sensors state
Header    header      # Standard header.
string[]  name        # Sensor name.
float64[] range       # Measured range [meters].
                      #   Note: A value of NO_OBJECT indicates that no objects
                      #         are detected within sensor range. 
                      #   Note: Values outside of the [min, max] and not equal
                      #         to NO_OBJECT should be descarded.
