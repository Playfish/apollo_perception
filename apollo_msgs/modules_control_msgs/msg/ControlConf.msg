float64 control_period
float64 max_planning_interval_sec
float64 max_planning_delay_threshold 

  # 0 - manual; 1 - auto_drive; 2 - speed_only; 3 - steer_only
modules_canbus_msgs/DrivingMode driving_mode

  # 0: stop, at first should stop, then receive pad msg to start
modules_control_msgs/DrivingAction action

float64 soft_estop_brake
modules_control_msgs/ControllerType[] active_controllers
int32 max_steering_percentage_allowed
float64 max_status_interval_sec

modules_control_msgs/LatControllerConf lat_controller_conf
modules_control_msgs/LonControllerConf lon_controller_conf

float64 trajectory_period
float64 chassis_period
float64 localization_period

float64 minimum_speed_resolution
