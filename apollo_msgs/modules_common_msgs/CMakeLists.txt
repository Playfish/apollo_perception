cmake_minimum_required(VERSION 2.8.3)
project(modules_common_msgs)

find_package(catkin REQUIRED COMPONENTS geometry_msgs message_generation std_msgs)

add_message_files(
  DIRECTORY msg
  FILES
  StatusPb.msg
  PathPoint.msg

  Point2D.msg
  Point3D.msg
  SLPoint.msg
  ErrorCode.msg
  PointENU.msg
  PointLLH.msg
  Quaternion.msg
  SpeedPoint.msg
  TurnSignal.msg
  FrenetFramePoint.msg
  Header.msg

  GnssStatus.msg
  SoundRequest.msg
  InsStatus.msg
  StreamStatus.msg
  VehicleSignal.msg
  Path.msg
  TrajectoryPoint.msg)

generate_messages(DEPENDENCIES geometry_msgs std_msgs)

catkin_package(
  CATKIN_DEPENDS geometry_msgs message_runtime std_msgs)
