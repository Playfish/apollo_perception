cmake_minimum_required(VERSION 2.8.3)
project(modules_routing_msgs)

find_package(catkin REQUIRED COMPONENTS geometry_msgs message_generation std_msgs modules_common_msgs modules_map_msgs)

add_message_files(
  DIRECTORY msg
  FILES
  ChangeLaneType.msg
  CurvePoint.msg
  CurveRange.msg
  Edge.msg
  EdgeDirectionType.msg
  Graph.msg
  Landmark.msg
  LaneSegment.msg
  LaneWaypoint.msg
  Measurement.msg
  Node.msg
  Passage.msg
  POI.msg
  RoadSegment.msg
  RoutingConfig.msg
  RoutingRequest.msg
  RoutingResponse.msg 
)

generate_messages(DEPENDENCIES geometry_msgs std_msgs modules_common_msgs modules_map_msgs) 

catkin_package(
  CATKIN_DEPENDS geometry_msgs message_runtime std_msgs modules_common_msgs modules_map_msgs)
