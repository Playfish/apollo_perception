/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#ifndef MODULES_MAP_HDMAP_HDMAP_H_
#define MODULES_MAP_HDMAP_HDMAP_H_

#include <memory>
#include <string>
#include <vector>

#include "common/macro.h"
//#include "common/proto/geometry.pb.h"
//Geometry
#include <modules_common_msgs/PointENU.h>
#include <modules_common_msgs/PointLLH.h>
#include <modules_common_msgs/Point2D.h>
#include <modules_common_msgs/Point3D.h>
#include <modules_common_msgs/Quaternion.h>
//end Geometry
#include "map/hdmap/hdmap_common.h"
#include "map/hdmap/hdmap_impl.h"

#include <modules_map_msgs/Crosswalk.h>
#include <modules_map_msgs/Junction.h>
#include <modules_map_msgs/Lane.h>
#include <modules_map_msgs/Overlap.h>
#include <modules_map_msgs/Road.h>
#include <modules_map_msgs/Signal.h>
#include <modules_map_msgs/StopSign.h>
#include <modules_map_msgs/YieldSign.h>
#include <modules_map_msgs/ClearArea.h>
#include <modules_map_msgs/SpeedBump.h>
//#include "map/proto/map_crosswalk.pb.h"
//#include "map/proto/map_junction.pb.h"
//#include "map/proto/map_lane.pb.h"
//#include "map/proto/map_overlap.pb.h"
//#include "map/proto/map_road.pb.h"
//#include "map/proto/map_signal.pb.h"
//#include "map/proto/map_stop_sign.pb.h"
//#include "map/proto/map_yield_sign.pb.h"
//#include "map/proto/map_clear_area.pb.h"
//#include "map/proto/map_speed_bump.pb.h"

/**
 * @namespace apollo::hdmap
 * @brief apollo::hdmap
 */
namespace apollo {
namespace hdmap {

/**
 * @class HDMap
 *
 * @brief High-precision map loader interface.
 */
class HDMap {
 public:
  /**
   * @brief load map from local file
   * @param map_filename path of map data file
   * @return 0:success, otherwise failed
   */
  int LoadMapFromFile(const std::string& map_filename);

  LaneInfoConstPtr GetLaneById(const modules_map_msgs::Id& id) const;
  JunctionInfoConstPtr GetJunctionById(const modules_map_msgs::Id& id) const;
  SignalInfoConstPtr GetSignalById(const modules_map_msgs::Id& id) const;
  CrosswalkInfoConstPtr GetCrosswalkById(const modules_map_msgs::Id& id) const;
  StopSignInfoConstPtr GetStopSignById(const modules_map_msgs::Id& id) const;
  YieldSignInfoConstPtr GetYieldSignById(const modules_map_msgs::Id& id) const;
  ClearAreaInfoConstPtr GetClearAreaById(const modules_map_msgs::Id& id) const;
  SpeedBumpInfoConstPtr GetSpeedBumpById(const modules_map_msgs::Id& id) const;
  OverlapInfoConstPtr GetOverlapById(const modules_map_msgs::Id& id) const;
  RoadInfoConstPtr GetRoadById(const modules_map_msgs::Id& id) const;

  /**
   * @brief get all lanes in certain range
   * @param point the central point of the range
   * @param distance the search radius
   * @param lanes store all lanes in target range
   * @return 0:success, otherwise failed
   */
  int GetLanes(const modules_common_msgs::PointENU& point, double distance,
               std::vector<LaneInfoConstPtr>* lanes) const;
  /**
   * @brief get all junctions in certain range
   * @param point the central point of the range
   * @param distance the search radius
   * @param junctions store all junctions in target range
   * @return 0:success, otherwise failed
   */
  int GetJunctions(const modules_common_msgs::PointENU& point, double distance,
                   std::vector<JunctionInfoConstPtr>* junctions) const;
  /**
   * @brief get all signals in certain range
   * @param point the central point of the range
   * @param distance the search radius
   * @param signals store all signals in target range
   * @return 0:success, otherwise failed
   */
  int GetSignals(const modules_common_msgs::PointENU& point, double distance,
                 std::vector<SignalInfoConstPtr>* signals) const;
  /**
   * @brief get all crosswalks in certain range
   * @param point the central point of the range
   * @param distance the search radius
   * @param crosswalks store all crosswalks in target range
   * @return 0:success, otherwise failed
   */
  int GetCrosswalks(const modules_common_msgs::PointENU& point, double distance,
                    std::vector<CrosswalkInfoConstPtr>* crosswalks) const;
  /**
   * @brief get all stop signs in certain range
   * @param point the central point of the range
   * @param distance the search radius
   * @param stop signs store all stop signs in target range
   * @return 0:success, otherwise failed
   */
  int GetStopSigns(const modules_common_msgs::PointENU& point, double distance,
                   std::vector<StopSignInfoConstPtr>* stop_signs) const;
  /**
   * @brief get all yield signs in certain range
   * @param point the central point of the range
   * @param distance the search radius
   * @param yield signs store all yield signs in target range
   * @return 0:success, otherwise failed
   */
  int GetYieldSigns(const modules_common_msgs::PointENU& point, double distance,
                    std::vector<YieldSignInfoConstPtr>* yield_signs) const;
  /**
   * @brief get all clear areas in certain range
   * @param point the central point of the range
   * @param distance the search radius
   * @param clear_areas store all clear areas in target range
   * @return 0:success, otherwise failed
   */
  int GetClearAreas(const modules_common_msgs::PointENU& point, double distance,
                    std::vector<ClearAreaInfoConstPtr>* clear_areas) const;
  /**
   * @brief get all speed bumps in certain range
   * @param point the central point of the range
   * @param distance the search radius
   * @param speed_bumps store all speed bumps in target range
   * @return 0:success, otherwise failed
   */
  int GetSpeedBumps(const modules_common_msgs::PointENU& point, double distance,
                    std::vector<SpeedBumpInfoConstPtr>* speed_bumps) const;
  /**
   * @brief get all roads in certain range
   * @param point the central point of the range
   * @param distance the search radius
   * @param roads store all roads in target range
   * @return 0:success, otherwise failed
   */
  int GetRoads(const modules_common_msgs::PointENU& point, double distance,
               std::vector<RoadInfoConstPtr>* roads) const;
  /**
   * @brief get nearest lane from target point,
   * @param point the target point
   * @param nearest_lane the nearest lane that match search conditions
   * @param nearest_s the offset from lane start point along lane center line
   * @param nearest_l the lateral offset from lane center line
   * @return 0:success, otherwise, failed.
   */
  int GetNearestLane(const modules_common_msgs::PointENU& point,
                     LaneInfoConstPtr* nearest_lane,
                     double* nearest_s, double* nearest_l) const;
  /**
   * @brief get the nearest lane within a certain range by pose
   * @param point the target position
   * @param distance the search radius
   * @param central_heading the base heading
   * @param max_heading_difference the heading range
   * @param nearest_lane the nearest lane that match search conditions
   * @param nearest_s the offset from lane start point along lane center line
   * @param nearest_l the lateral offset from lane center line
   * @return 0:success, otherwise, failed.
   */
  int GetNearestLaneWithHeading(const modules_common_msgs::PointENU& point,
                                const double distance,
                                const double central_heading,
                                const double max_heading_difference,
                                LaneInfoConstPtr* nearest_lane,
                                double* nearest_s, double* nearest_l) const;
  /**
   * @brief get all lanes within a certain range by pose
   * @param point the target position
   * @param distance the search radius
   * @param central_heading the base heading
   * @param max_heading_difference the heading range
   * @param nearest_lane all lanes that match search conditions
   * @return 0:success, otherwise, failed.
   */
  int GetLanesWithHeading(const modules_common_msgs::PointENU& point,
                          const double distance,
                          const double central_heading,
                          const double max_heading_difference,
                          std::vector<LaneInfoConstPtr>* lanes) const;
  /**
   * @brief get all road and junctions boundaries within certain range
   * @param point the target position
   * @param radius the search radius
   * @param road_boundaries the roads' boundaries
   * @param junctions the junctions' boundaries
   * @return 0:success, otherwise failed
   */
  int GetRoadBoundaries(const modules_common_msgs::PointENU& point, double radius,
                        std::vector<RoadROIBoundaryPtr>* road_boundaries,
                        std::vector<JunctionBoundaryPtr>* junctions) const;

 private:
  HDMapImpl impl_;
};

}  // namespace hdmap
}  // namespace apollo

#endif  // MODULES_MAP_HDMAP_HDMAP_H_
