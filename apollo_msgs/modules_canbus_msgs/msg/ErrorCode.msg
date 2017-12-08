#Error Code 
uint8 NO_ERROR = 0

uint8 CMD_NOT_IN_PERIOD = 1  # control cmd not in period

# car chassis report error, like steer, brake, throttle, gear fault
uint8 CHASSIS_ERROR = 2

uint8 MANUAL_INTERVENTION = 3  # human manual intervention

# receive car chassis can frame not in period
uint8 CHASSIS_CAN_NOT_IN_PERIOD = 4

uint8 UNKNOWN_ERROR = 5
