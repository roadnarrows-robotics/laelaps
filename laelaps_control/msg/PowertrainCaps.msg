### Laelaps laelaps_control/PowertrainCaps
#
# Robot base powertrain capabilities and specifications.
#
# Laelaps is 4WD, so there are 4 powertrains, grouped by the front and rear
# motor controllers. Laelaps ships symetrically. The user can easily alter the
# front and back mechanics (e.g. change rear tires to a larger size). It is 
# strongly recommended to not make the left-right powertrains unsymmetric.
#
# Powertrain: motor_controller - encoder - gear_box - drive_shaft - wheel - tire
#

string[]  name          # Powertrain name.
                        #  [left_front, right_front, left_rear, right_rear].
float64[] motor_max_rot # Motor maximum rotation [radians/second].
                        #   This value was determine by experimentation on
                        #   unloaded motors. The values vary somewhat.
float64[] gear_ratio    # Motor gear ratio.
float64[] resolution    # Number of encoder pulses per radian at drive shaft.
                        #   (post gear box).
float64[] tire_radius   # Wheel tire radius [meters].
                        #   Note: Tires are not rigid objects. The faster the
                        #         rotation, the more distortion occurs.
float64[] tire_width    # Wheel tire width [meters].
