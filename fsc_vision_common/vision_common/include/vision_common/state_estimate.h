#pragma once

#include <nav_msgs/Odometry.h>
#include <ros/time.h>
#include <Eigen/Dense>

#include "vision_common/gate_feature_array.h"
#include "vision_msgs/StateEstimate.h"

namespace vision_common {

struct StateEstimate {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  StateEstimate();

  StateEstimate(const vision_msgs::StateEstimate& state_estimate_msg);
  
  StateEstimate(const nav_msgs::Odometry& state_estimate_msg);

  virtual ~StateEstimate();

  vision_msgs::StateEstimate toRosMessage() const;
  
  void transformVelocityToWorldFrame();
  
  bool isValid() const;

  ros::Time timestamp;
  enum class CoordinateFrame
  {
    INVALID, WORLD, OPTITRACK, VISION, LOCAL
  } coordinate_frame;
  
  Eigen::Vector3d position;
  Eigen::Vector3d velocity;
  Eigen::Quaterniond orientation;
  vision_common::GateFeatureArray gates;

  Eigen::Vector3d bodyrates; // Body rates are represented in body coordinates
  double collective_thrust;
};

}  // namespace vision_common
