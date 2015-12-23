### Laelaps laelaps_control/AuxPwr message
#
# User controllable top deck auxilliary power out enable/disable.
#

string                    name  # Aux power name: [aux_batt_en, aux_5v_en]
industrial_msgs/TriState  state # Aux. power state  [DISABLED(0), ENABLED(1)]
