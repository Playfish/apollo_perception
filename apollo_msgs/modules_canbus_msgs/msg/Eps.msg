#Electrical Power Steering
#Type
uint8 NOT_AVAILABLE = 0
uint8 READY = 1
uint8 ACTIVE = 2
uint8 INVALID = 3

#changan: eps 2a0
bool is_eps_fail = 1

#eps 2a0 Type
uint8 eps_control_state     #for changan to control steering
float64 eps_driver_hand_torq #unit:Nm

bool is_steering_angle_valid
float64 steering_angle    #unit:degree
float64 steering_angle_spd #unit:degree/s

#byd sas 11f
bool is_trimming_status
bool is_calibration_status
bool is_failure_status
int32 allow_enter_autonomous_mode
int32 current_driving_mode

#lincoln steering 65
float64 steering_angle_cmd
float64 vehicle_speed
float64 epas_torque
bool steering_enabled
bool driver_override
bool driver_activity
bool watchdog_fault
bool channel_1_fault
bool channel_2_fault
bool calibration_fault
bool connector_fault

float64 timestamp_65

#lincoln version 7f
int32 major_version
int32 minor_version
int32 build_number
