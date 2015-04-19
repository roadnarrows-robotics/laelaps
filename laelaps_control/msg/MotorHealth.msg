### Laelaps laelaps_control/MotorHealth message

# Motor health.

string          name        # motor name
float32         temp        # motor temperature (C)
float32         voltage     # motor input voltage (Volts)
float32         current     # motor draw current (Amperes)
uint8           alarm       # motor or'ed alarm bits (see below)

### alarm bits 
uint8   ALARM_NONE        = 0      # no alarms
uint8   ALARM_CURRENT_MOT = 1      # motor over current
uint8   ALARM_VOLTAGE     = 2      # motor over/under voltage
uint8   ALARM_ESTOP       = 4      # motor emergency stopped
uint8   ALARM_TEMP        = 8      # motor over temperature
