# Turn Light Type
uint8 TURN_LIGHT_OFF = 0
uint8 TURN_LEFT_ON = 1
uint8 TURN_RIGHT_ON = 2
uint8 TURN_LIGHT_ON = 3

#Lamp Type
uint8 BEAM_OFF = 0
uint8 HIGH_BEAM_ON = 1
uint8 LOW_BEAM_ON = 2

#Lincoln Lamp Type
uint8 BEAM_NULL = 0
uint8 BEAM_FLASH_TO_PASS = 1
uint8 BEAM_HIGH = 2
uint8 BEAM_INVALID = 3

#Lincoln Wiper Type
uint8 WIPER_OFF = 0
uint8 WIPER_AUTO_OFF = 1
uint8 WIPER_OFF_MOVING = 2
uint8 WIPER_MANUAL_OFF = 3
uint8 WIPER_MANUAL_ON = 4
uint8 WIPER_MANUAL_LOW = 5
uint8 WIPER_MANUAL_HIGH = 6
uint8 WIPER_MIST_FLICK = 7
uint8 WIPER_WASH = 8
uint8 WIPER_AUTO_LOW = 9
uint8 WIPER_AUTO_HIGH = 10
uint8 WIPER_COURTESY_WIPE = 11
uint8 WIPER_AUTO_ADJUST = 12
uint8 WIPER_RESERVED = 13
uint8 WIPER_STALLED = 14
uint8 WIPER_NO_DATA = 15

#Lincoln Ambient Type
uint8 AMBIENT_DARK = 0
uint8 AMBIENT_LIGHT = 1
uint8 AMBIENT_TWILIGHT = 2
uint8 AMBIENT_TUNNEL_ON = 3
uint8 AMBIENT_TUNNEL_OFF = 4
uint8 AMBIENT_INVALID = 5
uint8 AMBIENT_NO_DATA = 7

#Turn Light Type
uint8 turn_light_type

#Lamp Type
uint8 lamp_type

bool is_brake_lamp_on

#byd switch 133
bool is_auto_light
int32 wiper_gear
int32 lotion_gear
bool is_horn_on

#lincoln misc 69
#Lincoln Lamp Type
uint8 lincoln_lamp_type

#Lincoln Wiper Type
uint8 lincoln_wiper

#Lincoln Ambient Type
#uint8 lincoln_ambient
