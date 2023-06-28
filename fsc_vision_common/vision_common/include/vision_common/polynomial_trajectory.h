#pragma once

#include <Eigen/Dense>
#include "vision_common/polynomial_curve.h"
#include "vision_msgs/PolynomialTrajectory.h"

namespace vision_common {

struct PolynomialTrajectory {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PolynomialTrajectory();

  PolynomialTrajectory(const std::array<double, 5>& coeffs_x,
                       const std::array<double, 5>& coeffs_y,
                       const std::array<double, 5>& coeffs_z);

  PolynomialTrajectory(const vision_msgs::PolynomialTrajectory& msg);

  ~PolynomialTrajectory();

  vision_msgs::PolynomialTrajectory toRosMessage() const;

  ros::Time timestamp;
  PolynomialCurve trajectory_x;
  PolynomialCurve trajectory_y;
  PolynomialCurve trajectory_z;
};

}