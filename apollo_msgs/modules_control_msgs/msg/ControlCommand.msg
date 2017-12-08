modules_common_msgs/Header header
  # target throttle in percentage [0, 100]
float64 throttle

  # target brake in percentage [0, 100]
float64 brake

  # target non-directional steering rate, in percentage of full scale per second [0, 100]
float64 steering_rate

  # target steering angle, in percentage of full scale [-100, 100]
float64 steering_target

  # parking brake engage. true: engaged
bool parking_brake

  # target speed, in km/h
float64 speed

  # target acceleration in m`s^-2
float64 acceleration

  # model reset
bool reset_model

  # engine on/off, true: engine on
bool engine_on_off

  # completion percentage of trajectory planned in last cycle
float64 trajectory_fraction

modules_canbus_msgs/DrivingMode driving_mode #deprecated = true
modules_canbus_msgs/GearPosition gear_location

modules_control_msgs/Debug debug
modules_common_msgs/VehicleSignal signal
modules_control_msgs/LatencyStats latency_stats
modules_control_msgs/PadMessage pad_msg

  # deprecated fields
bool left_turn #deprecated = true
bool right_turn  #deprecated = true
bool high_beam  #deprecated = true
bool low_beam  #deprecated = true
bool horn  #deprecated = true
modules_common_msgs/TurnSignal turnsignal  #deprecated = true
