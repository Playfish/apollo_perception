#A point in the map reference frame. The map defines an origin, whose
#coordinate is (0, 0, 0).
#Most modules, including localization, perception, and prediction, generate
#results based on the map reference frame.
#Currently, the map uses Universal Transverse Mercator (UTM) projection. See
#the link below for the definition of map origin.
#  https://en.wikipedia.org/wiki/Universal_Transverse_Mercator_coordinate_system
#The z field of PointENU can be omitted. If so, it is a 2D location and we do
#not care its height.

float64 x   #East from the origin, in meters.
float64 y   #North from the origin, in meters.
float64 z   #Up from the WGS-84 ellipsoid, in
                                          #meters.
