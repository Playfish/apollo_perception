cmake_minimum_required(VERSION 2.8.3)
project(modules_drivers_gnss_msgs)

find_package(catkin REQUIRED COMPONENTS geometry_msgs message_generation std_msgs modules_common_msgs)

add_message_files(
  DIRECTORY msg
  FILES
  Tcp.msg
  Udp.msg
  Imu.msg
  Ins.msg
  Gnss.msg
  GnssSolutionType.msg
  Ntrip.msg
  GPGGA.msg
  Config.msg
  Format.msg
  Serial.msg
  Stream.msg
  StreamType.msg
  DatumId.msg
  InsStat.msg
  InsType.msg
  GnssType.msg
  GnssRange.msg
  RangeInfo.msg
  InsStatus.msg
  InsStatusType.msg
  GnssBandID.msg
  PseudoType.msg
  GnssStatus.msg
  UbloxConfig.msg
  GnssBestPose.msg
  SolutionType.msg
  GlonassOrbit.msg
  GnssTimeType.msg
  KepplerOrbit.msg
  StreamStatus.msg
  device_config.msg
  NovatelConfig.msg
  Constellation.msg
  GnssEphemeris.msg
  InsStatusType.msg
  SolutionStatus.msg
  RtkSolutionType.msg
  BandObservation.msg
  EpochObservation.msg
  SatelliteObservation.msg

)

generate_messages(DEPENDENCIES geometry_msgs std_msgs modules_common_msgs)

catkin_package(
  CATKIN_DEPENDS geometry_msgs message_runtime std_msgs modules_common_msgs)
