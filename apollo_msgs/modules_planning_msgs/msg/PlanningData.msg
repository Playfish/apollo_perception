# next id: 17
# input
modules_localization_msgs/LocalizationEstimate adc_position #  optional 
modules_canbus_msgs/Chassis chassis #  optional 
modules_routing_msgs/RoutingResponse routing #  optional 
modules_common_msgs/TrajectoryPoint init_point #  optional 

modules_common_msgs/Path[] path  
SpeedPlan[] speed_plan   
STGraphDebug[] st_graph  
SLFrameDebug[] sl_frame 

modules_common_msgs/Header prediction_header 
SignalLightDebug signal_light  
