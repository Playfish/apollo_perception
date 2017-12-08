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

#ifndef MODULES_PERCEPTION_ONBOARD_HDMAP_INPUT_H_
#define MODULES_PERCEPTION_ONBOARD_HDMAP_INPUT_H_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

//#include "gtest/gtest_prod.h" //后续加入

#include "common/macro.h" 
#include "map/hdmap/hdmap.h"
#include "perception/lib/pcl_util/pcl_types.h"
#include "perception/obstacle/base/hdmap_struct.h"
#include "perception/obstacle/base/types.h"
#include <ros/ros.h>

namespace apollo {
namespace perception {

// Singleton HDMapInput, interfaces are thread-safe.
class HDMapInput {
 public:
  HDMapInput(ros::NodeHandle& nh);

  bool Init();

  // @brief: get roi polygon
  //         all points are in the world frame
  // 以velodyne为中心，60米为半径（配置文件得到），得到世界地图中的ROI（包括道路和路口信息）
  bool GetROI(const pcl_util::PointD& pointd, HdmapStructPtr* mapptr);

  // @brief: get nearest lane direction
  bool GetNearestLaneDirection(const pcl_util::PointD& pointd,
                               Eigen::Vector3d* lane_direction);

 private:
  void DownSampleBoundary(const modules_map_msgs::LineSegment* line,
                          PolygonDType* out_boundary_line) const;

  int MergeBoundaryJunction(
      const std::vector<apollo::hdmap::RoadROIBoundaryPtr>& boundaries,
      const std::vector<apollo::hdmap::JunctionBoundaryPtr>& junctions,
      HdmapStructPtr* mapptr);

  std::mutex mutex_;  // multi-thread init safe.

  //desclar ROS
  ros::NodeHandle nh_;
  std::string map_dir;
  std::string base_map_filename;
  std::string sim_map_filename;
  std::string routing_map_filename;
  std::string end_way_point_filename;

  std::string vehicle_config_path;
  double map_radius;
  int map_sample_step;
  bool use_ros_time;
//  FRIEND_TEST(HDMapInputTest, test_Init);  //TODO
//  FRIEND_TEST(HDMapInputTest, test_GetROI);//TODO

  DECLARE_SINGLETON(HDMapInput);
};

typedef typename std::shared_ptr<HDMapInput> HDMapInputPtr;

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_ONBOARD_HDMAP_INPUT_H_
