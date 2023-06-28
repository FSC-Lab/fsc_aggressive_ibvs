#pragma once

#include <Eigen/Dense>
#include <array>
#include "vision_msgs/PolynomialCurve.h"

namespace vision_common {

struct PolynomialCurve {
  PolynomialCurve()
      : PolynomialCurve(std::array<double, 5>{0.0, 0.0, 0.0, 0.0, 0.0}) {}

  PolynomialCurve(const std::array<double, 5>& coeffs);

  PolynomialCurve(const vision_msgs::PolynomialCurve& msg);

  ~PolynomialCurve() {}

  vision_msgs::PolynomialCurve toRosMessage() const;

  double x_s;
  double a;
  double b;
  double c;
  double y;
};

}  // namespace vision_common