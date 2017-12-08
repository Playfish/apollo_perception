#GearPosition
uint8 GEAR_NEUTRAL = 0
uint8 GEAR_DRIVE = 1
uint8 GEAR_REVERSE = 2
uint8 GEAR_PARKING = 3
uint8 GEAR_LOW = 4
uint8 GEAR_INVALID = 5
uint8 GEAR_NONE = 6

# tcu 268
bool is_shift_position_valid

  # chanan: tcu 268
#GearPosition
uint8 gear_state

  # lincoln gear 67
bool driver_override

#GearPosition 
uint8 gear_cmd
bool canbus_fault
