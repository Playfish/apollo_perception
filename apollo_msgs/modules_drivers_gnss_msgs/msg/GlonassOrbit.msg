#This message defines orbit parameters of GLONASS
modules_drivers_gnss_msgs/GnssType gnss_type  #[default = GLO_SYS]
uint32 slot_prn 
modules_drivers_gnss_msgs/GnssTimeType gnss_time_type  #[default = GLO_TIME]

#refer to GLONASS time and toc == toe
float64 toe

#must convert toe to UTC(+0) format and fullfil year, month...second_s,etc.
uint32 year 
uint32 month 
uint32 day 
uint32 hour 
uint32 minute 
float64 second_s 

#unit in meter
int32 frequency_no 

#GNSS week number    
uint32 week_num 

#GNSS week second in seconds
float64 week_second_s 

#frame broadcasted time
float64 tk

#clock correction(sec/sec),warning: set clock_offset = -TauN
float64 clock_offset 

#clock correction(sec/sec2),warning: set clock_drift = +GammaN
float64 clock_drift 

#Satellite health : 0=good,1=bad
uint32 health 

#unit in meter 
float64 position_x 
float64 position_y 
float64 position_z 

#unit in m/s
float64 velocity_x 
float64 velocity_y 
float64 velocity_z 

#unit in m/s2
float64 accelerate_x 
float64 accelerate_y 
float64 accelerate_z 
float64 infor_age 
