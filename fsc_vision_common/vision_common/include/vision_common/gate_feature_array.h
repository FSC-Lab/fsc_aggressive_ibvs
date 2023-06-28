#pragma once

#include <Eigen/Dense>
#include "vision_common/gate_feature.h"
#include "vision_msgs/GateFeatureArray.h"

namespace vision_common {

struct GateFeatureArray {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  GateFeatureArray();

  GateFeatureArray(const vision_msgs::GateFeatureArray& msg);

  virtual ~GateFeatureArray();

  void clear() { gates.clear(); }
  
  bool findGate(const std::string gate_id, vision_common::GateFeature& feature);

  vision_msgs::GateFeatureArray toRosMessage() const;

  ros::Time timestamp;
  std::vector<vision_common::GateFeature> gates;

};


}