### Laelaps laelaps_control/DutyCycle
#
# Powertrain duty cycles.
#
# Set powertrain target duty cycles. Any drives not specified remain at their
# respective current duty. Driving motors by specifying duty cycles is open
# loop. The motor encoders are ignored.
#

# powertrain duty cycles
Header    header      # Standard header.
string[]  names       # Powertrain names: [left_front, right_front,
                      #                   left_rear, right_rear].
float64[] duties      # Normalized duty cycles [-1.0, 1.0]
