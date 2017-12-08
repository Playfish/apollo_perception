# Temporary proto for map ROI used in perception module.
# Will be deprecated once the HOI map is ready.
# Next id: 9
 
modules_common_msgs/Header header  #Header.
modules_map_msgs/Header hdmap_header #HDMap header.

#Minimum x and y values in the map.
float64 origin_x  
float64 origin_y 

#Partition the map into cells of grid.
float64 grid_size  #in meters.
int32 num_rows  #repsect to x.
int32 num_columns  #respect to y.

#ROI regions on the grid.
PerceptionMapROIRegion[] region  
