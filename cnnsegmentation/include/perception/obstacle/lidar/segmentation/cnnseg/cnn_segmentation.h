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

#ifndef MODULES_PERCEPTION_OBSTACLE_LIDAR_SEGMENTATION_CNNSEG_CNN_SEGMENTATION_H_  // NOLINT
#define MODULES_PERCEPTION_OBSTACLE_LIDAR_SEGMENTATION_CNNSEG_CNN_SEGMENTATION_H_  // NOLINT

#include <memory>
#include <string>
#include <vector>

#include "caffe/caffe.hpp"

//#include "perception/obstacle/lidar/segmentation/cnnseg/proto/cnnseg.pb.h"
#include "modules_perception_cnnseg_msgs/CNNSegParam.h"
#include "modules_perception_cnnseg_msgs/NetworkParam.h"
#include "modules_perception_cnnseg_msgs/FeatureParam.h"

#include "common/log.h"
#include "perception/lib/base/timer.h"
#include "perception/lib/pcl_util/pcl_types.h"
#include "perception/obstacle/base/object.h"
#include "perception/obstacle/lidar/interface/base_segmentation.h"
#include "perception/obstacle/lidar/segmentation/cnnseg/cluster2d.h"
#include "perception/obstacle/lidar/segmentation/cnnseg/feature_generator.h"

#include <ros/ros.h>
namespace apollo {
namespace perception {

class CNNSegmentation : public BaseSegmentation {
 public:
  CNNSegmentation(ros::NodeHandle& nh);
  CNNSegmentation() : BaseSegmentation() {}
  ~CNNSegmentation() {}

  bool Init() override;

  bool Segment(const pcl_util::PointCloudPtr& pc_ptr,
               const pcl_util::PointIndices& valid_indices,
               const SegmentationOptions& options,
               std::vector<ObjectPtr>* objects) override;

  std::string name() const override { return "CNNSegmentation"; }

  float range() const { return range_; }
  int width() const { return width_; }
  int height() const { return height_; }

 private:
  bool GetConfigs(std::string* config_file, std::string* proto_file,
                  std::string* weight_file);
  // range of bird-view field (for each side)
  float range_ = 0.0;
  // number of cells in bird-view width
  int width_ = 0;
  // number of cells in bird-view height
  int height_ = 0;

  // paramters of CNNSegmentation
  modules_perception_cnnseg_msgs::CNNSegParam cnnseg_param_;
  // Caffe network object
  std::shared_ptr<caffe::Net<float>> caffe_net_;

  // bird-view raw feature generator
  std::shared_ptr<cnnseg::FeatureGenerator<float>> feature_generator_;

  // center offset prediction
  boost::shared_ptr<caffe::Blob<float>> instance_pt_blob_;
  // objectness prediction
  boost::shared_ptr<caffe::Blob<float>> category_pt_blob_;
  // fg probability prediction
  boost::shared_ptr<caffe::Blob<float>> confidence_pt_blob_;
  // object height prediction
  boost::shared_ptr<caffe::Blob<float>> height_pt_blob_;
  // raw features to be input into network
  boost::shared_ptr<caffe::Blob<float>> feature_blob_;

  // use all points of cloud to compute features
  bool use_full_cloud_ = false;

  // clustering model for post-processing
  std::shared_ptr<cnnseg::Cluster2D> cluster2d_;

  // timer
  Timer timer_;

  //ROS
  ros::NodeHandle nh_;
  double objectness_thresh;
  bool use_all_grids_for_clustering;
  double confidence_thresh;
  double height_thresh;
  int min_pts_num;
  bool use_full_cloud;
  int gpu_id;
  std::string instance_pt_blob;
  std::string category_pt_blob;
  std::string confidence_pt_blob;
  std::string height_pt_blob;
  std::string feature_blob;
  
  int point_cloud_range_;
  int min_height_;
  int max_height_;
  modules_perception_cnnseg_msgs::NetworkParam network_param;
  modules_perception_cnnseg_msgs::FeatureParam feature_param;
  
  DISALLOW_COPY_AND_ASSIGN(CNNSegmentation);
};

REGISTER_SEGMENTATION(CNNSegmentation);

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_LIDAR_SEGMENTATION_CNNSEG_CNN_SEGMENTATION_H_  // NOLINT
