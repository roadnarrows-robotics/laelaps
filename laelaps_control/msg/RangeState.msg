### Laelaps laelaps_control/RangeState message
#
# The state of all of the object detection range sensors.
# 

# sensors state
Header    header      # Standard header.
string[]  name        # Sensor name.
float64[] range       # Measured range [meters].
                      #   Note: A value of +Inf indicates that no objects
                      #         are detected within sensor range. 
                      #   Note: Values outside of the [min, max] and not equal
                      #         to +Inf should be descarded.
