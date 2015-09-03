### Laelaps laelaps_control/TofSensorCaps
#
# Robot base time-of-flight IR distance sensors capabilities and specifications.
#

string[]  name          # Sensor name: [front, left_front, left, left_rear,
                        #               rear, right_rear, right, right_front].
                        #   There are up to 8 time-of-flight sensors positioned
                        #   around the circumference of the robot base.
float64[] max_hz        # Maximum sense/read rate [hertz]. 
float64[] min_range     # Minimum sensed range [meters]. 
float64[] max_range     # Maximum sensed range [meters]. 
float64[] resolution    # Resolution [meters].
float64[] deadzone      # Tunable deadzone [meters].
                        #   Objects sensed in min_range-deadzone are ignored.
float64[] orientation   # Sensor center of orientation in x-y plane [radians].
                        #   Angle starts at 0 degrees straight forward, and
                        #   proceeds counter-clockwise. So 90 degrees is left,
                        #   180 degrees is the rear, etc
float64[] field_of_view # Detection field of view [radians].
                        #   Objects in the -field_of_view/2 to +field_of_view/2
                        #   are detectable.
