cmake_minimum_required(VERSION 2.8.3)
project(modules_common_configs_msgs)

find_package(catkin REQUIRED COMPONENTS geometry_msgs message_generation std_msgs modules_common_msgs)

add_message_files(
  DIRECTORY msg
  FILES
  Extrinsics.msg
  Transform.msg
  VehicleConfig.msg
  VehicleParam.msg
)

generate_messages(DEPENDENCIES geometry_msgs std_msgs modules_common_msgs)

catkin_package(
  CATKIN_DEPENDS geometry_msgs message_runtime std_msgs modules_common_msgs)
