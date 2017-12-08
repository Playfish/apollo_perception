#// next id: 16
modules_common_msgs/Header header #optional
float64 total_path_length  #optional in meters
float64 total_path_time   #optional in seconds

modules_common_msgs/TrajectoryPoint[] trajectory_point
EStop estop  

modules_common_msgs/PathPoint[] path_point  
Debug debug  

#is_replan == true mean replan triggered
bool is_replan #[default = false] 

# Specify trajectory gear
modules_canbus_msgs/GearPosition gear  
DecisionResult decision  
LatencyStats latency_stats  

# the routing used for current planning result
modules_common_msgs/Header routing_header  

# replaced by path_point
ADCPathPoint[] adc_path_point #[deprecated=true] 

# replaced by trajectory_point
ADCTrajectoryPoint[] adc_trajectory_point #[deprecated=true] 
modules_common_msgs/VehicleSignal signal #[deprecated=true] 
