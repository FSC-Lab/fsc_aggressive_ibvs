#pragma once

#include <Eigen/Dense>
#include "vision_common/normal_vector.h"
#include "vision_msgs/VisualFeature.h"

namespace vision_common {

struct VisualFeature {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  VisualFeature();

  VisualFeature(const NormalVector& normal_vector, const double& distance);

  VisualFeature(const Eigen::Quaterniond& q, const double& distance);

  VisualFeature(const Eigen::Vector3d& vec, const double& distance);

  VisualFeature(const Eigen::Vector3d& vec);

  VisualFeature(const std::string& id, const NormalVector& normal_vector,
                const double& distance);

  VisualFeature(const std::string& id, const Eigen::Quaterniond& q,
                const double& distance);

  VisualFeature(const std::string& id, const Eigen::Vector3d& vec,
                const double& distance);

  VisualFeature(const std::string& id, const Eigen::Vector3d& vec);

  VisualFeature(const vision_msgs::VisualFeature& msg);

  virtual ~VisualFeature();

  vision_msgs::VisualFeature toRosMessage() const;

  Eigen::Vector3d toPoint() const;
  
  bool isValid() const;

  std::string id;
  NormalVector normal_vector;
  double distance;
  double variance;
};

}  // namespace vision_common