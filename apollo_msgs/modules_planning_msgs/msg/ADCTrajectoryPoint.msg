#Deprecated: replaced by modules_common_msgs/TrajectoryPoint
float64 x    #  optional  in meters.
float64 y    #  optional  in meters.
float64 z    #  optional  height in meters.

float64 speed    #  optional  speed, in meters / second
float64 acceleration_s    #  optional  acceleration in s direction
float64 curvature    #  optional  curvature (k = 1/r), unit: (1/meters)

  #  change of curvature in unit s (dk/ds)
float64 curvature_change_rate  

  #  in seconds (relative_time = time_of_this_state - timestamp_in_header)
float64 relative_time  
float64 theta    #  optional  relative to absolute coordinate system

  #  calculated from the first point in this trajectory
float64 accumulated_s  

  #  in meters, reference to route SL-coordinate
float64 s #[deprecated = true] 

  #  in meters, reference to route SL-coordinate
float64 l  #[deprecated = true ]
