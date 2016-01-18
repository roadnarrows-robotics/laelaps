### Laelaps laelaps_control/MotorCtlrHealth message
#
# Motor controller health. For Laelaps, each controller controls two motors.
# 

# motor controller health
string          name        # Motor controller name: [front, rear].
float32         temp        # Motor controller temperature [C]
float32         voltage     # Battery input voltage [Volts]
Alarms          alarms      # Motor controller alarm statue
