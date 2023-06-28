#include "vision_common/visual_feature.h"

namespace vision_common {

VisualFeature::VisualFeature() : id(""), normal_vector(), distance(0.0) {}

VisualFeature::VisualFeature(const NormalVector& normal_vector, const double& distance)
    : id(""), normal_vector(normal_vector), distance(distance) {}

VisualFeature::VisualFeature(const Eigen::Quaterniond& q, const double& distance)
    : id(""), normal_vector(q), distance(distance) {}

VisualFeature::VisualFeature(const Eigen::Vector3d& vec, const double& distance)
    : id(""), normal_vector(vec), distance(distance) {}

VisualFeature::VisualFeature(const Eigen::Vector3d& vec)
    : id(""), normal_vector(vec), distance(vec.norm()) {}

VisualFeature::VisualFeature(const std::string& id, const NormalVector& normal_vector,
                           const double& distance)
    : id(id), normal_vector(normal_vector), distance(distance) {}

VisualFeature::VisualFeature(const std::string& id, const Eigen::Quaterniond& q,
                           const double& distance)
    : id(id), normal_vector(q), distance(distance) {}

VisualFeature::VisualFeature(const std::string& id, const Eigen::Vector3d& vec,
                           const double& distance)
    : id(id), normal_vector(vec), distance(distance) {}

VisualFeature::VisualFeature(const std::string& id, const Eigen::Vector3d& vec)
    : id(id), normal_vector(vec), distance(vec.norm()) {}

VisualFeature::VisualFeature(const vision_msgs::VisualFeature& msg) {
  id = msg.id;
  normal_vector = NormalVector(vision_common::geometryToEigen(msg.normal_vector));
  distance = msg.distance;
}

VisualFeature::~VisualFeature() {}

vision_msgs::VisualFeature VisualFeature::toRosMessage() const {
  vision_msgs::VisualFeature ros_msg;
  ros_msg.id = id;
  ros_msg.normal_vector = vision_common::eigenToGeometry(normal_vector.orientation);
  ros_msg.distance = distance;
  return ros_msg;
}

Eigen::Vector3d VisualFeature::toPoint() const {
  return normal_vector.getVec() * distance;
}

bool VisualFeature::isValid() const {
  if (!normal_vector.isValid()) {
    return false;
  }

  if (std::isnan(distance)) {
    return false;
  }

  return true;
}

}  // namespace vision_common