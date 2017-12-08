# ems 255
bool is_gas_pedal_error
  # ems 26a
bool is_gas_pedal_pressed_more  # more than auto mode gas torq
float64 gas_pedal_position  # manual gas default = 0
  # byd vtog 342
bool is_gas_valid  #default = false

  # lincoln throttle 63
float64 throttle_input
float64 throttle_cmd
float64 throttle_output
int32 watchdog_source
bool throttle_enabled
bool driver_override
bool driver_activity
bool watchdog_fault
bool channel_1_fault
bool channel_2_fault
bool connector_fault

  # lincoln throttleinfo 75
float64 accelerator_pedal
float64 accelerator_pedal_rate

  # lincoln version 7f
int32 major_version
int32 minor_version
int32 build_number
