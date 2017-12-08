/* Copyright 2017 The Apollo Authors. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
=========================================================================*/

#ifndef MODULES_MAP_HDMAP_HDMAP_COMMON_H_
#define MODULES_MAP_HDMAP_HDMAP_COMMON_H_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "common/math/aabox2d.h"
#include "common/math/aaboxkdtree2d.h"
#include "common/math/math_utils.h"
#include "common/math/polygon2d.h"
#include "common/math/vec2d.h"

//#include "map/proto/map_crosswalk.pb.h"
#include <modules_map_msgs/Crosswalk.h>

//#include "map/proto/map_id.pb.h"
#include <modules_map_msgs/Id.h>

//#include "map/proto/map_junction.pb.h"
#include <modules_map_msgs/Junction.h>

//#include "map/proto/map_lane.pb.h"
//#include <modules_map_msgs/LaneBoundary.h>
#include <modules_map_msgs/LaneSampleAssociation.h>
#include <modules_map_msgs/Lane.h>

//#include "map/proto/map_overlap.pb.h"
#include <modules_map_msgs/ObjectOverlapInfo.h>
#include <modules_map_msgs/Overlap.h>

//#include "map/proto/map_road.pb.h"
#include <modules_map_msgs/BoundaryEdge.h>
#include <modules_map_msgs/BoundaryPolygon.h>
#include <modules_map_msgs/RoadBoundary.h>
#include <modules_map_msgs/RoadROIBoundary.h>
#include <modules_map_msgs/RoadSection.h>
#include <modules_map_msgs/Road.h>

//#include "map/proto/map_signal.pb.h"
#include <modules_map_msgs/Subsignal.h>
#include <modules_map_msgs/Signal.h>

//#include "map/proto/map_stop_sign.pb.h"
#include <modules_map_msgs/StopSign.h>

//#include "map/proto/map_yield_sign.pb.h"
#include <modules_map_msgs/YieldSign.h>

//#include "map/proto/map_clear_area.pb.h"
#include <modules_map_msgs/ClearArea.h>

//#include "map/proto/map_speed_bump.pb.h"
#include <modules_map_msgs/SpeedBump.h>

/**
 * @namespace apollo::hdmap
 * @brief apollo::hdmap
 */
namespace apollo {
namespace hdmap {

template <class Object, class GeoObject>
class ObjectWithAABox {
 public:
  ObjectWithAABox(const apollo::common::math::AABox2d &aabox,
                  const Object *object, const GeoObject *geo_object,
                  const int id)
      : aabox_(aabox), object_(object), geo_object_(geo_object), id_(id) {}
  ~ObjectWithAABox() {}
  const apollo::common::math::AABox2d &aabox() const { return aabox_; }
  double DistanceTo(const apollo::common::math::Vec2d &point) const {
    return geo_object_->DistanceTo(point);
  }
  double DistanceSquareTo(const apollo::common::math::Vec2d &point) const {
    return geo_object_->DistanceSquareTo(point);
  }
  const Object *object() const { return object_; }
  const GeoObject *geo_object() const { return geo_object_; }
  int id() const { return id_; }

 private:
  apollo::common::math::AABox2d aabox_;
  const Object *object_;
  const GeoObject *geo_object_;
  int id_;
};

class LaneInfo;
class JunctionInfo;
class CrosswalkInfo;
class SignalInfo;
class StopSignInfo;
class YieldSignInfo;
class OverlapInfo;
class ClearAreaInfo;
class SpeedBumpInfo;

class HDMapImpl;

using LaneSegmentBox =
    ObjectWithAABox<LaneInfo, apollo::common::math::LineSegment2d>;
using LaneSegmentKDTree = apollo::common::math::AABoxKDTree2d<LaneSegmentBox>;

typedef std::shared_ptr<const OverlapInfo> OverlapInfoConstPtr;
class LaneInfo {
 public:
  explicit LaneInfo(const modules_map_msgs::Lane &lane);

  const modules_map_msgs::Id &id() const { return lane_.id; }
  const modules_map_msgs::Id &road_id() const { return road_id_; }
  const modules_map_msgs::Id &section_id() const { return section_id_; }
  const modules_map_msgs::Lane &lane() const { return lane_; }
  const std::vector<apollo::common::math::Vec2d> &points() const {
    return points_;
  }
  const std::vector<apollo::common::math::Vec2d> &unit_directions() const {
    return unit_directions_;
  }
  double Heading(const double s) const;
  const std::vector<double> &headings() const { return headings_; }
  const std::vector<apollo::common::math::LineSegment2d> &segments() const {
    return segments_;
  }
  const std::vector<double> &accumulate_s() const { return accumulated_s_; }
  const std::vector<OverlapInfoConstPtr> &overlaps() const { return overlaps_; }
  const std::vector<OverlapInfoConstPtr> &cross_lanes() const {
    return cross_lanes_;
  }
  const std::vector<OverlapInfoConstPtr> &signals() const { return signals_; }
  const std::vector<OverlapInfoConstPtr> &yield_signs() const {
    return yield_signs_;
  }
  const std::vector<OverlapInfoConstPtr> &stop_signs() const {
    return stop_signs_;
  }
  const std::vector<OverlapInfoConstPtr> &crosswalks() const {
    return crosswalks_;
  }
  const std::vector<OverlapInfoConstPtr> &junctions() const {
    return junctions_;
  }
  const std::vector<OverlapInfoConstPtr> &clear_areas() const {
    return clear_areas_;
  }
  const std::vector<OverlapInfoConstPtr> &speed_bumps() const {
    return speed_bumps_;
  }
  double total_length() const { return total_length_; }
  using SampledWidth = std::pair<double, double>;
  const std::vector<SampledWidth> &sampled_left_width() const {
    return sampled_left_width_;
  }
  const std::vector<SampledWidth> &sampled_right_width() const {
    return sampled_right_width_;
  }
  void GetWidth(const double s, double *left_width, double *right_width) const;
  double GetWidth(const double s) const;
  double GetEffectiveWidth(const double s) const;

  bool IsOnLane(const apollo::common::math::Vec2d &point) const;
  bool IsOnLane(const apollo::common::math::Box2d &box) const;

  modules_common_msgs::PointENU GetSmoothPoint(double s) const;
  double DistanceTo(const apollo::common::math::Vec2d &point) const;
  double DistanceTo(const apollo::common::math::Vec2d &point,
                    apollo::common::math::Vec2d *map_point, double *s_offset,
                    int *s_offset_index) const;
  modules_common_msgs::PointENU GetNearestPoint(
      const apollo::common::math::Vec2d &point, double *distance) const;
  bool GetProjection(const apollo::common::math::Vec2d &point,
                     double *accumulate_s, double *lateral) const;

 private:
  friend class HDMapImpl;
  friend class RoadInfo;
  void Init();
  void PostProcess(const HDMapImpl &map_instance);
  void UpdateOverlaps(const HDMapImpl &map_instance);
  double GetWidthFromSample(
      const std::vector<LaneInfo::SampledWidth> &samples, const double s) const;
  void CreateKDTree();
  void set_road_id(const modules_map_msgs::Id &road_id) { road_id_ = road_id; }
  void set_section_id(const modules_map_msgs::Id &section_id) { section_id_ = section_id; }

 private:
  const modules_map_msgs::Lane &lane_;
  std::vector<apollo::common::math::Vec2d> points_;
  std::vector<apollo::common::math::Vec2d> unit_directions_;
  std::vector<double> headings_;
  std::vector<apollo::common::math::LineSegment2d> segments_;
  std::vector<double> accumulated_s_;
  std::vector<std::string> overlap_ids_;
  std::vector<OverlapInfoConstPtr> overlaps_;
  std::vector<OverlapInfoConstPtr> cross_lanes_;
  std::vector<OverlapInfoConstPtr> signals_;
  std::vector<OverlapInfoConstPtr> yield_signs_;
  std::vector<OverlapInfoConstPtr> stop_signs_;
  std::vector<OverlapInfoConstPtr> crosswalks_;
  std::vector<OverlapInfoConstPtr> parking_spaces_;
  std::vector<OverlapInfoConstPtr> junctions_;
  std::vector<OverlapInfoConstPtr> clear_areas_;
  std::vector<OverlapInfoConstPtr> speed_bumps_;
  double total_length_ = 0.0;
  std::vector<SampledWidth> sampled_left_width_;
  std::vector<SampledWidth> sampled_right_width_;

  std::vector<LaneSegmentBox> segment_box_list_;
  std::unique_ptr<LaneSegmentKDTree> lane_segment_kdtree_;

  modules_map_msgs::Id road_id_;
  modules_map_msgs::Id section_id_;
};

class JunctionInfo {
 public:
  explicit JunctionInfo(const modules_map_msgs::Junction &junction);

  const modules_map_msgs::Id &id() const { return junction_.id; }
  const modules_map_msgs::Junction &junction() const { return junction_; }
  const apollo::common::math::Polygon2d &polygon() const { return polygon_; }

 private:
  void Init();

 private:
  const modules_map_msgs::Junction &junction_;
  apollo::common::math::Polygon2d polygon_;
};
using JunctionPolygonBox =
    ObjectWithAABox<JunctionInfo, apollo::common::math::Polygon2d>;
using JunctionPolygonKDTree =
    apollo::common::math::AABoxKDTree2d<JunctionPolygonBox>;

class SignalInfo {
 public:
  explicit SignalInfo(const modules_map_msgs::Signal &signal);

  const modules_map_msgs::Id &id() const { return signal_.id; }
  const modules_map_msgs::Signal &signal() const { return signal_; }
  const std::vector<apollo::common::math::LineSegment2d> &segments() const {
    return segments_;
  }

 private:
  void Init();

 private:
  const modules_map_msgs::Signal &signal_;
  std::vector<apollo::common::math::LineSegment2d> segments_;
};
using SignalSegmentBox =
    ObjectWithAABox<SignalInfo, apollo::common::math::LineSegment2d>;
using SignalSegmentKDTree =
    apollo::common::math::AABoxKDTree2d<SignalSegmentBox>;

class CrosswalkInfo {
 public:
  explicit CrosswalkInfo(const modules_map_msgs::Crosswalk &crosswalk);

  const modules_map_msgs::Id &id() const { return crosswalk_.id; }
  const modules_map_msgs::Crosswalk &crosswalk() const { return crosswalk_; }
  const apollo::common::math::Polygon2d &polygon() const { return polygon_; }

 private:
  void Init();

 private:
  const modules_map_msgs::Crosswalk &crosswalk_;
  apollo::common::math::Polygon2d polygon_;
};
using CrosswalkPolygonBox =
    ObjectWithAABox<CrosswalkInfo, apollo::common::math::Polygon2d>;
using CrosswalkPolygonKDTree =
    apollo::common::math::AABoxKDTree2d<CrosswalkPolygonBox>;

class StopSignInfo {
 public:
  explicit StopSignInfo(const modules_map_msgs::StopSign &stop_sign);

  const modules_map_msgs::Id &id() const { return stop_sign_.id; }
  const modules_map_msgs::StopSign &stop_sign() const { return stop_sign_; }
  const std::vector<apollo::common::math::LineSegment2d> &segments() const {
    return segments_;
  }

 private:
  void init();

 private:
  const modules_map_msgs::StopSign &stop_sign_;
  std::vector<apollo::common::math::LineSegment2d> segments_;
};
using StopSignSegmentBox =
    ObjectWithAABox<StopSignInfo, apollo::common::math::LineSegment2d>;
using StopSignSegmentKDTree =
    apollo::common::math::AABoxKDTree2d<StopSignSegmentBox>;

class YieldSignInfo {
 public:
  explicit YieldSignInfo(const modules_map_msgs::YieldSign &yield_sign);

  const modules_map_msgs::Id &id() const { return yield_sign_.id; }
  const modules_map_msgs::YieldSign &yield_sign() const { return yield_sign_; }
  const std::vector<apollo::common::math::LineSegment2d> &segments() const {
    return segments_;
  }

 private:
  void Init();

 private:
  const modules_map_msgs::YieldSign &yield_sign_;
  std::vector<apollo::common::math::LineSegment2d> segments_;
};
using YieldSignSegmentBox =
    ObjectWithAABox<YieldSignInfo, apollo::common::math::LineSegment2d>;
using YieldSignSegmentKDTree =
    apollo::common::math::AABoxKDTree2d<YieldSignSegmentBox>;

class ClearAreaInfo {
 public:
  explicit ClearAreaInfo(const modules_map_msgs::ClearArea &clear_area);

  const modules_map_msgs::Id &id() const { return clear_area_.id; }
  const modules_map_msgs::ClearArea &clear_area() const { return clear_area_; }
  const apollo::common::math::Polygon2d &polygon() const { return polygon_; }

 private:
  void Init();

 private:
  const modules_map_msgs::ClearArea &clear_area_;
  apollo::common::math::Polygon2d polygon_;
};
using ClearAreaPolygonBox =
    ObjectWithAABox<ClearAreaInfo, apollo::common::math::Polygon2d>;
using ClearAreaPolygonKDTree =
    apollo::common::math::AABoxKDTree2d<ClearAreaPolygonBox>;

class SpeedBumpInfo {
 public:
  explicit SpeedBumpInfo(const modules_map_msgs::SpeedBump &speed_bump);

  const modules_map_msgs::Id &id() const { return speed_bump_.id; }
  const modules_map_msgs::SpeedBump &speed_bump() const { return speed_bump_; }
  const std::vector<apollo::common::math::LineSegment2d> &segments() const {
    return segments_;
  }
 private:
  void Init();

 private:
  const modules_map_msgs::SpeedBump &speed_bump_;
  std::vector<apollo::common::math::LineSegment2d> segments_;
};
using SpeedBumpSegmentBox =
    ObjectWithAABox<SpeedBumpInfo, apollo::common::math::LineSegment2d>;
using SpeedBumpSegmentKDTree =
    apollo::common::math::AABoxKDTree2d<SpeedBumpSegmentBox>;

class OverlapInfo {
 public:
  explicit OverlapInfo(const modules_map_msgs::Overlap &overlap);

  const modules_map_msgs::Id &id() const { return overlap_.id; }
  const modules_map_msgs::Overlap &overlap() const { return overlap_; }
  const modules_map_msgs::ObjectOverlapInfo *get_object_overlap_info(const modules_map_msgs::Id &id) const;

 private:
  const modules_map_msgs::Overlap &overlap_;
};

class RoadInfo {
 public:
  explicit RoadInfo(const modules_map_msgs::Road &road);
  const modules_map_msgs::Id &id() const { return road_.id; }
  const modules_map_msgs::Road &road() const { return road_; }
  const std::vector<modules_map_msgs::RoadSection> &sections() const { return sections_; }

  const modules_map_msgs::Id &junction_id() const { return road_.junction_id; }
  bool has_junction_id() const { return !(road_.junction_id.id.empty());}//.has_junction_id(); }

  const std::vector<modules_map_msgs::RoadBoundary> &GetBoundaries() const;

 private:
  modules_map_msgs::Road road_;
  std::vector<modules_map_msgs::RoadSection> sections_;
  std::vector<modules_map_msgs::RoadBoundary> road_boundaries_;
};

using LaneInfoConstPtr = std::shared_ptr<const LaneInfo>;
using JunctionInfoConstPtr = std::shared_ptr<const JunctionInfo>;
using SignalInfoConstPtr = std::shared_ptr<const SignalInfo>;
using CrosswalkInfoConstPtr = std::shared_ptr<const CrosswalkInfo>;
using StopSignInfoConstPtr = std::shared_ptr<const StopSignInfo>;
using YieldSignInfoConstPtr = std::shared_ptr<const YieldSignInfo>;
using ClearAreaInfoConstPtr = std::shared_ptr<const ClearAreaInfo>;
using SpeedBumpInfoConstPtr = std::shared_ptr<const SpeedBumpInfo>;
using RoadInfoConstPtr = std::shared_ptr<const RoadInfo>;
using RoadROIBoundaryPtr = std::shared_ptr<modules_map_msgs::RoadROIBoundary>;

struct JunctionBoundary {
  JunctionInfoConstPtr junction_info;
};

using JunctionBoundaryPtr = std::shared_ptr<JunctionBoundary>;

}  // namespace hdmap
}  // namespace apollo

#endif  // MODULES_MAP_HDMAP_HDMAP_COMMON_H_
