cmake_minimum_required(VERSION 2.8.3)
project(modules_hmi_msgs)

find_package(catkin REQUIRED COMPONENTS geometry_msgs message_generation std_msgs modules_common_msgs modules_canbus_msgs)

add_message_files(
  DIRECTORY msg
  FILES
  Command.msg
  Component.msg
  Config.msg
  ConfigServer.msg
  ConfigServerHttps.msg
  ConfigStatus.msg
  HardwareStatus.msg
  HardwareStatusStatus.msg
  HMICommand.msg
  HMICommandChangeDrivingMode.msg
  ModuleStatus.msg
  ModuleStatusStatus.msg
  RuntimeStatus.msg
  SocketIORequest.msg
  ToolStatus.msg
  ToolStatusPlayingStatus.msg
  ToolStatusRecordingStatus.msg

)

generate_messages(DEPENDENCIES geometry_msgs std_msgs modules_common_msgs modules_canbus_msgs)

catkin_package(
  CATKIN_DEPENDS geometry_msgs message_runtime std_msgs modules_common_msgs modules_canbus_msgs)
