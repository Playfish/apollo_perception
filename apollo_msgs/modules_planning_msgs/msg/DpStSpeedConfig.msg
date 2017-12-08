float64 total_path_length  #[ default = 0.1 ] optional
float64 total_time   #[ default = 3.0 ] optional
int32 matrix_dimension_s   #[ default = 100 ] optional
int32 matrix_dimension_t  #[ default = 10 ] optional

float64 speed_weight   #[ default = 0.0 ] optional
float64 accel_weight   #[ default = 10.0 ] optional
float64 jerk_weight  #[ default = 10.0 ] optional
float64 obstacle_weight   #[ default = 1.0 ] optional
float64 reference_weight   #[ default = 0.0 ] optional
float64 go_down_buffer   #[ default = 5.0 ] optional
float64 go_up_buffer   #[ default = 5.0 ] optional

  # obstacle cost config
float64 default_obstacle_cost   #[ default = 1e10 ] optional
float64 obstacle_cost_factor   #[ default = -300 ] optional

  # speed cost config
float64 default_speed_cost   #[ default = 1.0 ] optional
float64 exceed_speed_penalty  #[ default = 10.0 ] optional
float64 low_speed_penalty   #[ default = 2.5 ] optional

  # accel cost config
float64 accel_penalty    #[ default = 2.0 ] optional
float64 decel_penalty   #[ default = 2.0 ] optional

  # jerk cost config
float64 positive_jerk_coeff   #[ default = 1.0 ] optional
float64 negative_jerk_coeff   #[ default = 300.0 ] optional

  # other constraint
float64 max_acceleration   #[ default = 4.5 ] optional
float64 max_deceleration   #[ default = -4.5 ] optional

StBoundaryConfig st_boundary_config  # optional

float64 keep_clear_cost_factor   #[ default = 1000.0 ] optional
