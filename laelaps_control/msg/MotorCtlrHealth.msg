### Laelaps laelaps_control/MotorCtlrHealth message

# Motor controller health. For Laelaps, each controller controls two motors.

string          name        # motor controller name
float32         temp        # motor controller temperature (C)
float32         voltage     # motor input voltage (Volts)
uint8           alarm       # motor or'ed alarm bits (see below)

### alarm bits 
uint8   ALARM_NONE              = 0      # no alarms
uint8   ALARM_OVER_CURRENT_MOT1 = 1      # motor 1 over current
uint8   ALARM_OVER_CURRENT_MOT2 = 2      # motor 2 over current
uint8   ALARM_ESTOP             = 4      # emergency stopped
uint8   ALARM_TEMP              = 8      # over temperature
uint8   ALARM_MAIN_BATT_HIGH    = 16     # main battery input too high
uint8   ALARM_MAIN_BATT_LOW     = 32     # main battery input too low
uint8   ALARM_LOGIC_BATT_HIGH   = 64     # logic battery input too high
uint8   ALARM_LOGIC_BATT_LOW    = 128    # logic battery input too low
