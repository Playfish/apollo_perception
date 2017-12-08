
#Priority Level
uint8 LOW = 0
uint8 MEDIUM = 1
uint8 HIGH = 2

#Mode
uint8 SAY = 0
uint8 BEEP = 1

Header header

#Priority of sound request
uint8 priority

#Type of sound requesting
uint8 mode

# Words to say in SAY mode
string words
