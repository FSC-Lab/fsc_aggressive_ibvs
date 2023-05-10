#pragma once

#include <ros/ros.h>
#include <Eigen/Dense>

namespace rc {

class RcMonitor {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  RcMonitor() : RcMonitor(ros::NodeHandle(), ros::NodeHandle("~")) {}

  ~RcMonitor();

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber rc_sub_;
  
  Eigen::Vector3d position_increment_;
  double heading_increment_;

  double monitor_frequency_;

}

}