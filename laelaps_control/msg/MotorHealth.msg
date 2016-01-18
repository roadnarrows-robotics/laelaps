### Laelaps laelaps_control/MotorHealth message
#
# Motor health.
# 

# motor health
string          name      # Motor name: [left_front, right_front
                          #              left_rear, right_rear]
float32         temp      # Motor temperature [C].
float32         voltage   # Motor input voltage [Volts].
float32         current   # Motor draw current [Amperes].
Alarms          alarms    # Motor alarm state
