### Laelaps laelaps_control/Gpio message
#
# User controllable GPIO pins.
#

uint8 pin       # GPIO pin
uint8 state     # GPIO state or direction

# Pin Ids
uint8 GPIO_PIN_MIN = 2  # GPIO minimum pin number.
uint8 GPIO_PIN_MAX = 8  # GPIO maximum pin number.

# States
uint8 OFF = 0       # off (low)
uint8 ON  = 1       # on (high)

# Direction
uint8 IN  = 0       # input
uint8 OUT = 1       # output
