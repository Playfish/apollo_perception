cmake_minimum_required(VERSION 2.8.3)
project(modules_monitor_msgs)

find_package(catkin REQUIRED COMPONENTS geometry_msgs message_generation std_msgs modules_common_msgs modules_common_adapters_msgs)

add_message_files(
  DIRECTORY msg
  FILES
  HardwareStatus.msg
  HardwareStatusMap.msg
  HardwareStatusStatus.msg
  ModuleMonitorConf.msg
  ModuleMonitorConfModuleConf.msg
  ModuleStatus.msg
  ModuleStatusMap.msg
  SystemStatus.msg
)

generate_messages(DEPENDENCIES geometry_msgs std_msgs modules_common_msgs modules_common_adapters_msgs)

catkin_package(
  CATKIN_DEPENDS geometry_msgs message_runtime std_msgs modules_common_msgs modules_common_adapters_msgs)
