
#next id :28

bool engine_started

#Engine speed in RPM.
float32 engine_rpm

#Vehicle Speed in meters per second.
float32 speed_mps

#Vehicle odometer in meters.
float32 odometer_m

#Fuel range in meters.
int32 fuel_range_m

#Real throttle location in [%], ranging from 0 to 100.
float32 throttle_percentage

#Real brake location in [%], ranging from 0 to 100.
float32 brake_percentage

#Real gear location.
#optional int32 gear_location = 10 deprecated use enum replace this [id 23]
#Real steering location in [%], ranging from 0 to 100.
float32 steering_percentage

#Applied steering torque in [Nm].
float32 steering_torque_nm

#Parking brake status.
bool parking_brake


bool wiper
bool disengage_status
#Driving Mode 
modules_canbus_msgs/DrivingMode driving_mode

#Error Code
modules_canbus_msgs/ErrorCode error_code

#Gear Position
modules_canbus_msgs/GearPosition gear_location

#timestamp for steering module
float64 steering_timestamp  #In seconds, with 1e-6 accuracy

#chassis also needs it own sending timestamp
modules_common_msgs/Header header

int32 chassis_error_mask

modules_common_msgs/VehicleSignal signal

#Light signals.
bool high_beam_signal
bool low_beam_signal
bool left_turn_signal
bool right_turn_signal
bool horn
