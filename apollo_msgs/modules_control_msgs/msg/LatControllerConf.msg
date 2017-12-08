float64 ts             # sample time (dt) 0.01 now, configurable
  # preview window n, preview time = preview window * ts
int32 preview_window 
float64 cf 
float64 cr             # N/rad
int32 mass_fl 
int32 mass_fr 
int32 mass_rl 
int32 mass_rr 
float64 eps           # converge threshold for lqr solver
float64[] matrix_q      # matrix_q size = 4 + preview_window
int32 cutoff_freq    # cutoff frequency
int32 mean_filter_window_size   # window size of mean filter
  # for a normal car, it should be in range[16, 18]

int32 max_iteration   # maximum iteration for lqr solve
float64 max_lateral_acceleration   # limit aggressive steering
modules_control_msgs/LatGainScheduler lat_err_gain_scheduler 
modules_control_msgs/LatGainScheduler heading_err_gain_scheduler 
