### Laelaps laelaps_control/Alarms message
#
# The alarms message is contained within other laelaps_control messages to
# provide a unified alarm framework.
# 

# alarm state
bool    is_critical   # does [not] have critical alarms
uint32  alarms        # Alarms or'ed bits (see below)
uint32  warnings      # Warnings or'ed bits (see below)

### alarm bits 
uint32  ALARM_NONE                  = 0     # no alarms
uint32  ALARM_GEN                   = 1     # general, non-specific alarm 
uint32  ALARM_ESTOP                 = 2     # emergency stopped
uint32  ALARM_BATT                  = 4     # battery low alarm
uint32  ALARM_TEMP                  = 8     # temperature out-of-range alarm
uint32  ALARM_NO_RSRC               = 16    # no resource operational alarm
uint32  ALARM_SUBSYS                = 32    # subsystem(s) alarmed
uint32  ALARM_MOTCTLR_BATT_V_HIGH   = 256   # motor ctlr battery over volts
uint32  ALARM_MOTCTLR_BATT_V_LOW    = 512   # motor ctlr battery under volts
uint32  ALARM_MOTCTLR_LOGIC_V_HIGH  = 1024  # motor controller logic over volts
uint32  ALARM_MOTCTLR_LOGIC_V_LOW   = 2048  # motor controller logic under volts
uint32  ALARM_MOT_OVER_CUR          = 4096  # motor over current alarm
uint32  ALARM_MOT_FAULT             = 8192  # motor faulted alarm

# warning bits
uint32  WARN_NONE                 = 0     # no warnings
uint32  WARN_BATT                 = 4     # battery low warning
uint32  WARN_TEMP                 = 8     # temperature out-of-range warning
uint32  WARN_SUBSYS               = 32    # warnings on subsystem(s)
uint32  WARN_MOTCTLR_BATT_V_HIGH  = 256   # motor ctlr battery over volts
uint32  WARN_MOTCTLR_BATT_V_LOW   = 512   # motor ctlr battery under volts
uint32  WARN_MOT_OVER_CUR         = 4096  # motor over current warning
