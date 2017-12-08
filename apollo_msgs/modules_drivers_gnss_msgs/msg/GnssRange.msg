# All range measurements at a time.

modules_common_msgs/Header header 

  # The time of range measurement, seconds since the GPS epoch (Jan 6, 1980).
float64 measurement_time   # In seconds.

modules_drivers_gnss_msgs/RangeInfo[] ranges 
