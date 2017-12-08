# Electrical Parking Brake
#PBrake Type
uint8 PBRAKE_OFF = 0
uint8 PBRAKE_TRANSITION = 1
uint8 PBRAKE_ON = 2
uint8 PBRAKE_FAULT = 3

  # epb 256
bool is_epb_error
bool is_epb_released

  # byd epb 218
int32 epb_status

  # lincoln brakeinfo 74
#PBrake Type
uint8 parking_brake_status
