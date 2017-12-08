#Type
uint8 FORWARD = 0
uint8 BACKWARD = 1
uint8 STANDSTILL = 2
uint8 INVALID = 3

#esp 277
bool is_vehicle_standstill

#esp 218
bool is_vehicle_spd_valid
float64 vehicle_spd #unit:m/s
#esp 208
bool is_wheel_spd_rr_valid
#Type
uint8 wheel_direction_rr
float64 wheel_spd_rr
bool is_wheel_spd_rl_valid

#Type
uint8 wheel_direction_rl
float64 wheel_spd_rl
bool is_wheel_spd_fr_valid

#Type
uint8 wheel_direction_fr
float64 wheel_spd_fr
bool is_wheel_spd_fl_valid

#Type
uint8 wheel_direction_fl
float64 wheel_spd_fl

#byd esp 222
bool is_yaw_rate_valid
float64 yaw_rate
float64 yaw_rate_offset

#byd esp 223
bool is_ax_valid
float64 ax
float64 ax_offset
bool is_ay_valid
float64 ay
float64 ay_offset

#lincoln accel 6b
float64 lat_acc
float64 long_acc
float64 vert_acc

#lincoln gyro 6c
float64 roll_rate

#lincoln brakeinfo 74
float64 acc_est

#lincoln wheelspeed 6a
float64 timestamp_sec
