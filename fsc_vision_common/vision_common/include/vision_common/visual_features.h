#pragma once

#include <Eigen/Dense>
#include "vision_common/visual_feature.h"
#include "vision_msgs/VisualFeatures.h"

namespace vision_common {

struct VisualFeatures {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  VisualFeatures();

  VisualFeatures(const ros::Time& timestamp,
                 const std::vector<vision_common::VisualFeature>& features);

  VisualFeatures(const ros::Time& timestamp,
                 const vision_common::VisualFeature& feature);

  VisualFeatures(const std::vector<vision_common::VisualFeature>& features);

  VisualFeatures(const vision_common::VisualFeature& feature);

  VisualFeatures(const vision_msgs::VisualFeatures& msg);

  virtual ~VisualFeatures();

  bool findFeature(const std::string id, vision_common::VisualFeature& feature);

  vision_msgs::VisualFeatures toRosMessage() const;

  bool isValid() const;

  bool empty() const { return features.empty(); }

  size_t size() const { return features.size(); }

  void clear() { features.clear(); }
  
  void resize(const size_t size) { features.resize(size); }

  ros::Time timestamp;
  std::vector<vision_common::VisualFeature> features;
};

}  // namespace vision_common