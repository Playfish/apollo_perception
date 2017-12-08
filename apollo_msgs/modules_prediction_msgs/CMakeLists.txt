cmake_minimum_required(VERSION 2.8.3)
project(modules_prediction_msgs)

find_package(catkin REQUIRED COMPONENTS geometry_msgs message_generation std_msgs modules_common_msgs modules_perception_config_manager_msgs)

add_message_files(
  DIRECTORY msg
  FILES
  Feature.msg
  FnnVehicleModel.msg
  Lane.msg
  LaneFeature.msg
  LaneGraph.msg
  LanePoint.msg
  LaneSegment.msg
  LaneSequence.msg
  Layer.msg
  LayerActivationFunc.msg
  Matrix.msg
  NearbyObstacle.msg
  ObstacleConf.msg
  ObstacleConfEvaluatorType.msg
  ObstacleConfObstacleStatus.msg
  ObstacleConfPredictorType.msg
  PredictionConf.msg
  PredictionObstacle.msg
  PredictionObstacles.msg
  Trajectory.msg
  Vector.msg 
)

generate_messages(DEPENDENCIES geometry_msgs std_msgs modules_common_msgs modules_perception_config_manager_msgs) 

catkin_package(
  CATKIN_DEPENDS geometry_msgs message_runtime std_msgs modules_common_msgs modules_perception_config_manager_msgs)
