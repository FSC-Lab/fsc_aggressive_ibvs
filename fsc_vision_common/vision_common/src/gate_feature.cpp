#include "vision_common/gate_feature.h"

namespace vision_common {

GateFeature::GateFeature()
    : gate_id(""),
      center(),
      corners(),
      translation(Eigen::Vector3d::Zero()),
      rotation(Eigen::Quaterniond::Identity()),
      width(0.0),
      height(0.0) {}

GateFeature::GateFeature(const Eigen::Vector3d& center, const int num_corners)
    : gate_id(""),
      center(center),
      translation(Eigen::Vector3d::Zero()),
      rotation(Eigen::Quaterniond::Identity()) {
  for (int i = 0; i < num_corners; ++i) {
    corners.push_back(center);
  }
}

GateFeature::GateFeature(const std::string& gate_id,
                         const Eigen::Vector3d& center,
                         const std::vector<Eigen::Vector3d>& corner_vector,
                         const Eigen::Vector3d& translation,
                         const Eigen::Quaterniond& rotation)
    : gate_id(gate_id),
      center(center),
      translation(translation),
      rotation(rotation) {
  int size = corner_vector.size();
  corners.resize(size);
  for (int i = 0; i < size; ++i) {
    std::stringstream ss;
    ss << i;
    corners[i] = vision_common::VisualFeature(ss.str(), corner_vector[i]);
  }
}

GateFeature::GateFeature(const std::string& gate_id,
                         const Eigen::Vector3d& center,
                         const std::vector<Eigen::Vector3d>& corner_vector)
    : GateFeature(gate_id, center, corner_vector, Eigen::Vector3d::Zero(),
                  Eigen::Quaterniond::Identity()) {}

GateFeature::GateFeature(const std::string& gate_id,
                         const Eigen::Vector3d& position,
                         const Eigen::Quaterniond& orientation,
                         const double width, const double height)
    : gate_id(gate_id),
      center(position),
      translation(position),
      rotation(orientation),
      width(width),
      height(height) {
  double w_h = 0.5 * width;
  double h_h = 0.5 * height;
  // corner in local frame
  std::vector<Eigen::Vector3d> corner_vector;
  Eigen::Vector3d pt_ll(-h_h, w_h, 0);
  Eigen::Vector3d pt_lr(-h_h, -w_h, 0);
  Eigen::Vector3d pt_ur(h_h, -w_h, 0);
  Eigen::Vector3d pt_ul(h_h, w_h, 0);
  corner_vector.push_back(pt_ll);
  corner_vector.push_back(pt_lr);
  corner_vector.push_back(pt_ur);
  corner_vector.push_back(pt_ul);

  int size = corner_vector.size();
  corners.resize(size);
  for (int i = 0; i < size; ++i) {
    std::stringstream ss;
    ss << i;
    Eigen::Vector3d corner_pos = orientation * corner_vector[i] + position;
    corners[i] = vision_common::VisualFeature(ss.str(), corner_pos);
  }
}

GateFeature::GateFeature(const vision_msgs::GateFeature& msg) {
  gate_id = msg.gate_id;
  center = vision_common::VisualFeature(msg.center);

  int size = msg.corners.size();
  corners.resize(size);
  for (int i = 0; i < size; ++i) {
    corners[i] = vision_common::VisualFeature(msg.corners[i]);
  }

  translation = geometryToEigen(msg.pose.pose.position);
  rotation = geometryToEigen(msg.pose.pose.orientation);
  width = msg.width;
  height = msg.height;
}

GateFeature::~GateFeature() {}

vision_msgs::GateFeature GateFeature::toRosMessage() const {
  vision_msgs::GateFeature msg;

  msg.gate_id = gate_id;
  msg.center = center.toRosMessage();
  for (int i = 0; i < corners.size(); ++i) {
    msg.corners.push_back(corners[i].toRosMessage());
  }

  msg.pose.pose.position = vectorToPoint(eigenToGeometry(translation));
  msg.pose.pose.orientation = eigenToGeometry(rotation);
  msg.width = width;
  msg.height = height;
  return msg;
}

std::vector<Eigen::Vector3d> GateFeature::getCornerPoints() const {
  std::vector<Eigen::Vector3d> corner_points;
  for (int i = 0; i < corners.size(); ++i) {
    corner_points.push_back(corners[i].toPoint());
  }

  return corner_points;
}

}  // namespace vision_common