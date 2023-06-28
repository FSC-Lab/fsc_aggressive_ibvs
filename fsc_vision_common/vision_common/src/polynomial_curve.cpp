#include "vision_common/polynomial_curve.h"

namespace vision_common {

PolynomialCurve::PolynomialCurve(const std::array<double, 5>& coeffs) {
  x_s = coeffs[0];
  a = coeffs[1];
  b = coeffs[2];
  c = coeffs[3];
  y = coeffs[4];
}

PolynomialCurve::PolynomialCurve(const vision_msgs::PolynomialCurve& msg) {
  x_s = msg.x_s;
  a = msg.a;
  b = msg.b;
  c = msg.c;
  y = msg.y;
}

vision_msgs::PolynomialCurve PolynomialCurve::toRosMessage() const {
  vision_msgs::PolynomialCurve msg;
  msg.x_s = x_s;
  msg.a = a;
  msg.b = b;
  msg.c = c;
  msg.y = y;
  return msg;
}

}  // namespace vision_common