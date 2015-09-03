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
uint8           alarms    # Motor or'ed alarm bits (see below).

### alarm bits 
uint8 MOT_ALARM_NONE    = 0   # no alarms
uint8 MOT_ALARM_CURRENT = 1   # motor over current
uint8 MOT_ALARM_VOLTAGE = 2   # motor over/under voltage
uint8 MOT_ALARM_ESTOP   = 4   # motor emergency stopped
uint8 MOT_ALARM_TEMP    = 8   # motor over temperature
