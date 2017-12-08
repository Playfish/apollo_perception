cmake_minimum_required(VERSION 2.8.3)
project(modules_planning_msgs)

find_package(catkin REQUIRED COMPONENTS geometry_msgs message_generation std_msgs modules_common_msgs modules_routing_msgs modules_canbus_msgs modules_localization_msgs modules_perception_config_manager_msgs)

add_message_files(
  DIRECTORY msg
  FILES
  Debug.msg
  EStop.msg
  MainStop.msg
  TaskType.msg
  SpeedPlan.msg
  TaskStats.msg
  ADCSignals.msg
  MainCruise.msg
  ObjectStop.msg
  RuleConfig.msg
  SLBoundary.msg
  TargetLane.msg
  MainParking.msg
  ObjectAvoid.msg
  ObjectNudge.msg
  ObjectYield.msg
  SignalDebug.msg
  ADCPathPoint.msg
  LatencyStats.msg
  MainDecision.msg
  MainNotReady.msg
  ObjectFollow.msg
  ObjectIgnore.msg
  PlanningData.msg
  SLFrameDebug.msg
  STGraphDebug.msg
  ADCTrajectory.msg
  DecisionResult.msg
  MainChangeLane.msg
  ObjectDecision.msg
  ObjectOvertake.msg
  ObjectSidePass.msg
  PlanningConfig.msg
  QpSplineConfig.msg
  StopReasonCode.msg
  DpStSpeedConfig.msg
  EMPlannerConfig.msg
  ObjectDecisions.msg
  ObjectNudgeType.msg
  QpStSpeedConfig.msg
  DpPolyPathConfig.msg
  RuleConfigRuleId.msg
  SignalLightDebug.msg
  StBoundaryConfig.msg
  MainEmergencyStop.msg
  QpPiecewiseConfig.msg
  SpiralCurveConfig.msg
  ADCTrajectoryPoint.msg
  MainChangeLaneType.msg
  ObjectDecisionType.msg
  QpSplinePathConfig.msg
  MainMissionComplete.msg
  ADCSignalsSignalType.msg
  StGraphBoundaryDebug.msg
  STGraphKernelCuiseRef.msg
  EmergencyStopHardBrake.msg
  STGraphKernelFollowRef.msg
  STGraphSpeedConstraint.msg
  EmergencyStopCruiseToStop.msg
  PlanningConfigPlannerType.msg
  MainEmergencyStopReasonCode.msg
  StGraphBoundaryDebugStBoundaryType.msg
  QpSplineReferenceLineSmootherConfig.msg
)

generate_messages(DEPENDENCIES geometry_msgs std_msgs modules_common_msgs modules_routing_msgs modules_canbus_msgs modules_localization_msgs modules_perception_config_manager_msgs) 

catkin_package(
  CATKIN_DEPENDS geometry_msgs message_runtime std_msgs modules_common_msgs modules_routing_msgs modules_canbus_msgs modules_localization_msgs modules_perception_config_manager_msgs)
