#pragma once

#include <ros/duration.h>
#include <Eigen/Dense>
#include "vision_msgs/TrajectoryPoint.h"
#include "vision_common/conversion_common.h"

namespace vision_common {

struct TrajectoryPoint {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  TrajectoryPoint();
  TrajectoryPoint(const vision_msgs::TrajectoryPoint& msg);
  virtual ~TrajectoryPoint();

  vision_msgs::TrajectoryPoint toRosMessage() const;

  ros::Duration time_from_start;
  double length_from_start;
  Eigen::Vector3d tangent;
  double distance_to_gate;
  double distance_rate;
  std::string gate_id;
  
  // Pose
  Eigen::Vector3d position;
  Eigen::Quaterniond orientation;

  // Linear derivatives
  Eigen::Vector3d velocity;
  Eigen::Vector3d acceleration;
  Eigen::Vector3d jerk;
  Eigen::Vector3d snap;

  // Angular derivatives
  Eigen::Vector3d bodyrates;
  Eigen::Vector3d angular_acceleration;
  Eigen::Vector3d angular_jerk;
  Eigen::Vector3d angular_snap;

  // Heading angle with respect to world frame [rad]
  double heading;
  double heading_rate;
  double heading_acceleration;

  double collective_thrust;  // [m/s^2]
};

}  // namespace vision_common
