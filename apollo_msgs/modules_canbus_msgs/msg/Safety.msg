#Driving Mode
uint8 COMPLETE_MANUAL = 0  # human drive
uint8 COMPLETE_AUTO_DRIVE = 1
uint8 AUTO_STEER_ONLY = 2  # only steer
uint8 AUTO_SPEED_ONLY = 3  # include throttle and brake

    # security mode when manual intervention happens, only response status
uint8 EMERGENCY_MODE = 4

# ip 270
bool is_driver_car_door_close
  # sas 50
bool is_driver_buckled

  # byd sws 4a8
int32 emergency_button

  # qirui:403
  # when car chassis error, like system fault, or warning report
bool has_error
bool is_motor_invertor_fault
bool is_system_fault
bool is_power_battery_fault
bool is_motor_invertor_over_temperature
bool is_small_battery_charge_discharge_fault
int32 driving_mode

  # lincoln misc 69
bool is_passenger_door_open
bool is_rearleft_door_open
bool is_rearright_door_open
bool is_hood_open
bool is_trunk_open
bool is_passenger_detected
bool is_passenger_airbag_enabled
bool is_passenger_buckled

  # lincoln tirepressure 71
int32 front_left_tire_press
int32 front_right_tire_press
int32 rear_left_tire_press
int32 rear_right_tire_press

#Driving Mode
uint8 car_driving_mode
