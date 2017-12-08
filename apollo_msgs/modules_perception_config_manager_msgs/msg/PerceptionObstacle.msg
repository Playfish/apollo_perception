int32 id  #obstacle ID.
Point position  #obstacle position in the world coordinate
                              #system.
float64 theta  #heading in the world coordinate system.
Point velocity  #obstacle velocity.

#Size of obstacle bounding box.
float64 length  #obstacle length.
float64 width  #obstacle width.
float64 height  #obstacle height.

Point[] polygon_point  #obstacle corner points.

#duration of an obstacle since detection in s.
float64 tracking_time  
PerceptionObstacleType type  #obstacle type
float64 timestamp  #GPS time in seconds.

#Just for offline debuging, onboard will not fill this field.
#Format like : [x0, y0, z0, x1, y1, z1...]
float64[] point_cloud  # [packed = true]

float64 confidence  # [default = 1.0]
