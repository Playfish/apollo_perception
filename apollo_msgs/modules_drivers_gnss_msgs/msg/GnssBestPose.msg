modules_common_msgs/Header header
float64 measurement_time  # In seconds.
modules_drivers_gnss_msgs/SolutionStatus sol_status
modules_drivers_gnss_msgs/SolutionType sol_type
float64 latitude  # in degrees
float64 longitude  # in degrees
float64 height_msl  # height above mean sea level in meters
float32 undulation  # undulationheight_wgs84 - height_msl
modules_drivers_gnss_msgs/DatumId datum_id  # datum id number
float32 latitude_std_dev  # latitude standard deviation (m)
float32 longitude_std_dev  # longitude standard deviation (m)
float32 height_std_dev  # height standard deviation (m)
string base_station_id  # base station id
float32 differential_age  # differential position age (sec)
float32 solution_age  # solution age (sec)
uint32 num_sats_tracked  # number of satellites tracked
uint32 num_sats_in_solution  # number of satellites used in solution
uint32 num_sats_l1  # number of L1/E1/B1 satellites used in solution
uint32 num_sats_multi  # number of multi-frequency satellites used in solution
uint32 reserved  # reserved
uint32 extended_solution_status  # extended solution status - OEMV and
                                                  # greater only
uint32 galileo_beidou_used_mask
uint32 gps_glonass_used_mask
