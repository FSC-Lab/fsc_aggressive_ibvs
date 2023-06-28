#pragma once

#include <geometry_msgs/PoseWithCovariance.h>
#include <Eigen/Dense>
#include <sstream>
#include "vision_common/visual_feature.h"
#include "vision_msgs/GateFeature.h"

namespace vision_common {

struct GateFeature {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  GateFeature();

  GateFeature(const Eigen::Vector3d& center, const int num_corners = 0);

  GateFeature(const std::string& gate_id,
              const Eigen::Vector3d& center,
              const std::vector<Eigen::Vector3d>& corner_vector,
              const Eigen::Vector3d& translation,
              const Eigen::Quaterniond& rotation);

  GateFeature(const std::string& gate_id,
              const Eigen::Vector3d& center,
              const std::vector<Eigen::Vector3d>& corner_vector);

  GateFeature(const std::string& gate_id,
              const Eigen::Vector3d& position,
              const Eigen::Quaterniond& orientation,
              const double width,
              const double height);

  GateFeature(const vision_msgs::GateFeature& msg);

  virtual ~GateFeature();

  vision_msgs::GateFeature toRosMessage() const;

  std::vector<Eigen::Vector3d> getCornerPoints() const;

  std::string gate_id;
  vision_common::VisualFeature center;
  std::vector<vision_common::VisualFeature> corners;
  Eigen::Vector3d translation;
  Eigen::Quaterniond rotation;
  double width;
  double height;
};

}  // namespace vision_common