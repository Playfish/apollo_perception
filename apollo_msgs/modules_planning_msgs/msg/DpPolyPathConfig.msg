uint32 sample_points_num_each_level = 1 #[default = 9]  optional
float64 step_length_max = 2 #[default = 15.0]  optional
float64 step_length_min = 3 #[default = 8.0]  optional
float64 lateral_sample_offset = 4 #[default = 0.5]  optional
float64 lateral_adjust_coeff = 5 #[default = 0.5]  optional
  # Trajectory Cost Config
float64 eval_time_interval = 6 #[default = 0.1]  optional
float64 path_resolution = 7 #[default = 0.1]  optional
float64 obstacle_ignore_distance = 8 #[default = 20.0]  optional
float64 obstacle_collision_distance = 9 #[default = 0.2]  optional
float64 obstacle_risk_distance = 10 #[default = 2.0]  optional
float64 obstacle_collision_cost = 11 #[default = 1e3]  optional
