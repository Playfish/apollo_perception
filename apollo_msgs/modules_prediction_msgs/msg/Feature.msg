# Obstacle ID
int32 id   #   optional 

# Obstacle features
modules_common_msgs/Point3D position   #   optional 
modules_common_msgs/Point3D velocity  #   optional 
modules_common_msgs/Point3D acceleration  #   optional 
float64 velocity_heading    #   optional 
float64 speed   #   optional 
float64 acc   #   optional 
float64 theta   #   optional 
float64 length   #   optional 
float64 width   #   optional 
float64 height   #   optional 
float64 tracking_time   #   optional 
float64 timestamp   #   optional     

# Obstacle type-specific features
Lane lane   #   optional 

# Obstacle tracked features
modules_common_msgs/Point3D t_position    #   optional 
modules_common_msgs/Point3D t_velocity   #   optional 
float64 t_velocity_heading   #   optional 
float64 t_speed   #   optional 
modules_common_msgs/Point3D t_acceleration   #   optional 
float64 t_acc  #   optional 

bool is_still #[default = true]     optional 
