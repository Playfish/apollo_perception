cmake_minimum_required(VERSION 2.8.3)
project(modules_perception_config_manager_msgs)

find_package(catkin REQUIRED COMPONENTS geometry_msgs message_generation std_msgs modules_common_msgs modules_map_msgs)

add_message_files(
  DIRECTORY msg
  FILES
  PerceptionMapROI.msg
  PerceptionMapROIRegion.msg
  PerceptionObstacle.msg
  PerceptionObstacles.msg
  PerceptionObstacleType.msg
  Point.msg
  TrafficLight.msg
  TrafficLightColor.msg
  TrafficLightDetection.msg
)

generate_messages(DEPENDENCIES geometry_msgs std_msgs modules_common_msgs modules_map_msgs)

catkin_package(
  CATKIN_DEPENDS geometry_msgs message_runtime std_msgs modules_common_msgs modules_map_msgs)
