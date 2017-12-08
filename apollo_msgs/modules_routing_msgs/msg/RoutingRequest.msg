modules_common_msgs/Header header #optional

# at least two points. The first is start point, the end is final point.
# The routing must go through each point in waypoint.
LaneWaypoint[] waypoint  
LaneSegment[] blacklisted_lane 
string[] blacklisted_road  
bool broadcast #optional [default = true];
