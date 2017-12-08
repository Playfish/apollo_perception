cmake_minimum_required(VERSION 2.8.3)
project(modules_canbus_msgs)

find_package(catkin REQUIRED COMPONENTS geometry_msgs message_generation std_msgs modules_common_msgs)

add_message_files(
  DIRECTORY msg
  FILES
  Ems.msg
  Epb.msg
  Eps.msg
  Esp.msg
  Gas.msg
  Gear.msg
  Brake.msg
  Light.msg
  Safety.msg
  ChassisDetailType.msg
  Chassis.msg
  ErrorCode.msg
  DrivingMode.msg
  GearPosition.msg
  Battery.msg
  License.msg
  BasicInfo.msg
  CanbusConf.msg
  VehicleSpd.msg
  Deceleration.msg
  ChassisDetail.msg
  EmsType.msg
  CheckResponseSignal.msg
  VehicleParameter.msg
  VehicleParameterVehicleBrand.msg
  CANCardParameter.msg
  CANCardBrand.msg
  CANCardType.msg
  CANChannelId.msg
)

generate_messages(DEPENDENCIES geometry_msgs std_msgs modules_common_msgs)

catkin_package(
  CATKIN_DEPENDS geometry_msgs message_runtime std_msgs modules_common_msgs)
