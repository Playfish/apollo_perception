
# controller param
float64 ts   # longitudinal controller sampling time

float64 brake_deadzone  
float64 throttle_deadzone  
float64 speed_controller_input_limit  
float64 station_error_limit  
float64 preview_window  
float64 standstill_acceleration  

modules_control_msgs/PidConf station_pid_conf  
modules_control_msgs/PidConf low_speed_pid_conf  
modules_control_msgs/PidConf high_speed_pid_conf  
float64 switch_speed   # low/high speed controller switch speed

modules_control_msgs/FilterConf pitch_angle_filter_conf  

modules_control_msgs/ControlCalibrationTable calibration_table  
