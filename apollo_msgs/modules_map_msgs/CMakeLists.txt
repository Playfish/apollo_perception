cmake_minimum_required(VERSION 2.8.3)
project(modules_map_msgs)

find_package(catkin REQUIRED COMPONENTS geometry_msgs message_generation std_msgs modules_common_msgs)

add_message_files(
  DIRECTORY msg
  FILES
  BoundaryEdge.msg
  BoundaryEdgeType.msg
  BoundaryPolygon.msg
  ClearArea.msg
  ClearAreaOverlapInfo.msg
  Crosswalk.msg
  CrosswalkOverlapInfo.msg
  Curve.msg
  CurveSegment.msg
  Header.msg
  Id.msg
  Junction.msg
  JunctionOverlapInfo.msg
  Lane.msg
  LaneBoundary.msg
  LaneBoundaryType.msg
  LaneBoundaryTypeType.msg
  LaneDirection.msg
  LaneOverlapInfo.msg
  LaneSampleAssociation.msg
  LaneTurn.msg
  LaneType.msg
  LineSegment.msg
  Map.msg
  ObjectOverlapInfo.msg
  Overlap.msg
  ParkingSpaceOverlapInfo.msg
  Polygon.msg
  Projection.msg
  Road.msg
  RoadBoundary.msg
  RoadROIBoundary.msg
  RoadSection.msg
  Signal.msg
  SignalOverlapInfo.msg
  SignalType.msg
  SpeedBump.msg
  SpeedBumpOverlapInfo.msg
  StopSign.msg
  StopSignOverlapInfo.msg
  Subsignal.msg
  SubsignalType.msg
  YieldOverlapInfo.msg
  YieldSign.msg

)

generate_messages(DEPENDENCIES geometry_msgs std_msgs modules_common_msgs)

catkin_package(
  CATKIN_DEPENDS geometry_msgs message_runtime std_msgs modules_common_msgs)
