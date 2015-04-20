### Laelaps laelaps_control/Gpio message
#
# User controllable GPIO pins.
#

uint8 pin       # GPIO pin
uint8 state     # GPIO state or direction

# Pin Ids
uint8 X_GPIO_1 = 1  # TBD GPIO pin 1
uint8 Y_GPIO_2 = 2  # TBD GPIO pin 2

# States
uint8 OFF = 0       # off (low)
uint8 ON  = 1       # on (high)

# Direction
uint8 IN  = 0       # input
uint8 OUT = 1       # output
