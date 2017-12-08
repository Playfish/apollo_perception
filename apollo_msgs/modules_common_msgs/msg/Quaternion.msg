#A unit quaternion that represents a spatial rotation. See the link below for
#details.
#  https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation
#The scalar part qw can be omitted. In this case, qw should be calculated by
#  qw = sqrt(1 - qx * qx - qy * qy - qz * qz).

float64 qx
float64 qy
float64 qz
float64 qw

