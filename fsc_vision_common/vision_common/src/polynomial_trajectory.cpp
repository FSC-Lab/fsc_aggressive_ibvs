#include "vision_common/polynomial_trajectory.h"

namespace vision_common {

PolynomialTrajectory::PolynomialTrajectory() : timestamp(ros::Time::now()) {}

PolynomialTrajectory::PolynomialTrajectory(
    const std::array<double, 5>& coeffs_x,
    const std::array<double, 5>& coeffs_y,
    const std::array<double, 5>& coeffs_z)
    : timestamp(ros::Time::now()),
      trajectory_x(coeffs_x),
      trajectory_y(coeffs_y),
      trajectory_z(coeffs_z) {}

PolynomialTrajectory::PolynomialTrajectory(
    const vision_msgs::PolynomialTrajectory& msg) {
  timestamp = msg.header.stamp;
  trajectory_x = PolynomialCurve(msg.trajectory_x);
  trajectory_y = PolynomialCurve(msg.trajectory_y);
  trajectory_z = PolynomialCurve(msg.trajectory_z);
}

PolynomialTrajectory::~PolynomialTrajectory() {}

vision_msgs::PolynomialTrajectory PolynomialTrajectory::toRosMessage() const {
  vision_msgs::PolynomialTrajectory msg;
  msg.header.stamp = timestamp;
  msg.header.frame_id = "";
  msg.trajectory_x = trajectory_x.toRosMessage();
  msg.trajectory_y = trajectory_y.toRosMessage();
  msg.trajectory_z = trajectory_z.toRosMessage();
  return msg;
}

}  // namespace vision_common