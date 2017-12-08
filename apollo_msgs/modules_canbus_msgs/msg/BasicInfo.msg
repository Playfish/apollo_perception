#Type
uint8 OFF = 0
uint8 ACC = 1
uint8 ON = 2
uint8 START = 3
uint8 INVALID = 4

#GpsQuality 
uint8 FIX_NO = 0
uint8 FIX_2D = 1
uint8 FIX_3D = 2
uint8 FIX_INVALID = 3

bool is_auto_mode
#Type
uint8 power_state

bool is_air_bag_deployed
float64 odo_meter  # odo meter, unit:km
float64 drive_range  # the meter left when drive continuously, unit:km
bool is_system_error
bool is_human_interrupt  # human interrupt

  # lincoln misc 69
bool acc_on_button   # acc on button pressed
bool acc_off_button 
bool acc_res_button 
bool acc_cancel_button 
bool acc_on_off_button 
bool acc_res_cancel_button 
bool acc_inc_spd_button 
bool acc_dec_spd_button 
bool acc_inc_gap_button 
bool acc_dec_gap_button 
bool lka_button 
bool canbus_fault 

  # lincoln gps 6d
float64 latitude 
float64 longitude 
bool gps_valid 

  # lincoln gps 6e
int32 year 
int32 month 
int32 day 
int32 hours 
int32 minutes 
int32 seconds 
float64 compass_direction 
float64 pdop 
bool is_gps_fault 
bool is_inferred 

  # lincoln gps 6f
float64 altitude 
float64 heading 
float64 hdop 
float64 vdop 

#GpsQuality 
uint8 quality 
int32 num_satellites 
float64 gps_speed 
