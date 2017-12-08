#This message defines one epoch observation on a certain time of a receiver
#Unique id to a certain receiver
#0 for rover, otherwise for baser, supporting multi-baser mode
uint32 receiver_id

modules_drivers_gnss_msgs/GnssTimeType gnss_time_type #[default = GPS_TIME]
uint32 gnss_week
float64 gnss_second_s

#Baser observation should be binded with coordinates
#unit in meter
float64 position_x

#unit in meter
float64 position_y

#unit in meter
float64 position_z

#Health indicator: 0 for healthy while 1 for bad observation
uint32 health_flag #[default = 0]

#Number of observed satellites
uint32 sat_obs_num

#Group of observed satellite observation
SatelliteObservation[] sat_obs
