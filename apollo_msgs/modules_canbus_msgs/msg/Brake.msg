#HSA Status Type
uint8 HSA_INACTIVE = 0
uint8 HSA_FINDING_GRADIENT = 1
uint8 HSA_ACTIVE_PRESSED = 2
uint8 HSA_ACTIVE_RELEASED = 3
uint8 HSA_FAST_RELEASE = 4
uint8 HSA_SLOW_RELEASE = 5
uint8 HSA_FAILED = 6
uint8 HSA_UNDEFINED = 7

#HSA Mode Type
uint8 HSA_OFF = 0
uint8 HSA_AUTO = 1
uint8 HSA_MANUAL = 2
uint8 HSA_MODE_UNDEFINED = 3

#ems 255
bool is_brake_pedal_pressed#only manual brake
#esp 277
bool is_brake_force_exist # no matter auto mode brake or manual brake
bool is_brake_over_heat

bool is_hand_brake_on #hand brake
float64 brake_pedal_position

#byd vtog 342
bool is_brake_valid

#lincoln brake 61
float64 brake_input
float64 brake_cmd
float64 brake_output
bool boo_input
bool boo_cmd
bool boo_output
bool watchdog_applying_brakes
int32 watchdog_source
bool brake_enabled
bool driver_override
bool driver_activity
bool watchdog_fault
bool channel_1_fault
bool channel_2_fault
bool boo_fault
bool connector_fault

#lincoln brakeinfo 74
float64 brake_torque_req

#HSA Status Type 
uint8 hsa_status

float64 brake_torque_act
#HSA Mode Type 
uint8 hsa_mode
float64 wheel_torque_act

#lincoln version 7f
int32 major_version
int32 minor_version
int32 build_number
