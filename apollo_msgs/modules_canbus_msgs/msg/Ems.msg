# Engine Manager System
  # ems 26a
bool is_engine_acc_available  # for changan to control car
bool is_engine_acc_error   # for changan to control car

  # ems 265
modules_canbus_msgs/EmsType engine_state
float64 max_engine_torq_percent  # for engine torq control, unit:%
float64 min_engine_torq_percent  # for engine torq control, unit:%
int32 base_engine_torq_constant  # for engine torq control, unit:Nm

  # ems 255
bool is_engine_speed_error
float64 engine_speed

  # byd vtog 241
int32 engine_torque
  # byd vtog 242
bool is_over_engine_torque

  # lincoln mkz throttleinfo 75
float64 engine_rpm
