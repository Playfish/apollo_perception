#Car center point is car reference point, i.e., center of rear axle.
float64 front_edge_to_center
float64 back_edge_to_center
float64 left_edge_to_center
float64 right_edge_to_center

float64 length
float64 width
float64 height

float64 min_turn_radius
float64 max_acceleration
float64 max_deceleration

#The following items are used to compute trajectory constraints in planning.
#vehicle max steer angle
float64 max_steer_angle

#vehicle max steer rate; how fast can the steering wheel turn.
float64 max_steer_angle_rate

#ratio between the turn of steering wheel and the turn of wheels
float64 steer_ratio

#the distance between the front and back wheels
float64 wheel_base

#Tire effective rolling radius (vertical distance between the wheel center
#and the ground).
float64 wheel_rolling_radius
