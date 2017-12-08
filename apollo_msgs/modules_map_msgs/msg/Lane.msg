# A lane is part of a roadway, that is designated for use by a single line of vehicles.
# Most public roads (include highways) have more than two lanes.

Id id 

#Central lane as reference trajectory, not necessary to be the geometry central.
Curve central_curve 

#Lane boundary curve.
LaneBoundary left_boundary 
LaneBoundary right_boundary 

#in meters.
float64 length 

#Speed limit of the lane, in meters per second.
float64 speed_limit

Id[] overlap_id

#All lanes can be driving into (or from).
Id[] predecessor_id
Id[] successor_id

#Neighbor lanes on the same direction.
Id[] left_neighbor_forward_lane_id
Id[] right_neighbor_forward_lane_id 
LaneTurn turn 

Id[] left_neighbor_reverse_lane_id 
Id[] right_neighbor_reverse_lane_id 

Id junction_id 

#Association between central point to closest boundary.
LaneSampleAssociation[] left_sample 
LaneSampleAssociation[] right_sample 

LaneDirection direction 
