### Laelaps laelaps_control/AmbientLightSensorCaps
#
# Robot base ambient light sensors capabilities and specifications.
#

string[]  name          # Sensor name. [front, left_front, left, left_rear,
                        #               rear, right_rear, right, right_front].
                        #   There are up to 8 ambient light sensors positioned
                        #   around the circumference of the robot base.
float64[] max_hz        # Maximum sense/read rate [Hertz]. 
float64[] min_illum     # Minimum sensed illumination [lux]. 
float64[] max_illum     # Maximum sensed illumination [lux]. 
float64[] resolution    # Resolution [lux].
float64[] orientation   # Sensor center of orientation in x-y plane [radians].
                        #   Angle starts at 0 degrees straight forward, and
                        #   proceeds counter-clockwise. So 90 degrees is left,
                        #   180 degrees is at the rear, etc.
float64[] field_of_view # Detection field of view [radians].
                        #   The illumination in the -field_of_view/2 to
                        #   +field_of_view/2 is calculated.
