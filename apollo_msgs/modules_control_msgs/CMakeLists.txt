cmake_minimum_required(VERSION 2.8.3)
project(modules_control_msgs)

find_package(catkin REQUIRED COMPONENTS geometry_msgs message_generation std_msgs modules_common_msgs modules_canbus_msgs)

add_message_files(
  DIRECTORY msg
  FILES
  Debug.msg
  PidConf.msg
  InputDebug.msg
  TurnSignal.msg
  FilterConf.msg
  PadMessage.msg
  ControlConf.msg
  LatencyStats.msg
  DrivingAction.msg
  ControlCommand.msg
  ControllerType.msg
  LatGainScheduler.msg
  LatControllerConf.msg
  LonControllerConf.msg
  SimpleLateralDebug.msg
  LatGainSchedulerInfo.msg
  ControlCalibrationInfo.msg
  ControlCalibrationTable.msg
  SimpleLongitudinalDebug.msg
)

generate_messages(DEPENDENCIES geometry_msgs std_msgs modules_common_msgs modules_canbus_msgs)

catkin_package(
  CATKIN_DEPENDS geometry_msgs message_runtime std_msgs modules_common_msgs modules_canbus_msgs)
