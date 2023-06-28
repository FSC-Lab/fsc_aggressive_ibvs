#include "trajectory_generation_helper/circle_trajectory_helper.h"

#include <vision_common/math_common.h>
#include <vision_common/trajectory_point.h>

namespace trajectory_generation_helper {

namespace circles {

vision_common::Trajectory computeHorizontalCircleTrajectory(
    const Eigen::Vector3d center, const double radius, const double speed,
    const double phi_start, const double phi_end,
    const double sampling_frequency) {
  /*
   * We use a coordinate system with x to the front, y to the left, and z up.
   * When setting phi_start = 0.0 the start point will be at
   * (radius, 0, 0) + center
   * If phi_end > phi_start the trajectory is going counter clock wise
   * otherwise it is going clockwise
   */

  vision_common::Trajectory trajectory;
  trajectory.trajectory_type =
      vision_common::Trajectory::TrajectoryType::GENERAL;

  const double phi_total = phi_end - phi_start;
  const double direction = phi_total / fabs(phi_total);
  const double omega = direction * fabs(speed / radius);
  const double angle_step = fabs(omega / sampling_frequency);

  for (double d_phi = 0.0; d_phi < fabs(phi_total); d_phi += angle_step) {
    const double phi = phi_start + direction * d_phi;
    const double cos_phi = cos(phi);
    const double sin_phi = sin(phi);
    vision_common::TrajectoryPoint point;
    point.time_from_start = ros::Duration(fabs(d_phi / omega));
    point.position = radius * Eigen::Vector3d(cos_phi, sin_phi, 0.0) + center;
    point.velocity = radius * omega * Eigen::Vector3d(-sin_phi, cos_phi, 0.0);
    point.acceleration =
        radius * pow(omega, 2.0) * Eigen::Vector3d(-cos_phi, -sin_phi, 0.0);
    point.jerk =
        radius * pow(omega, 3.0) * Eigen::Vector3d(sin_phi, -cos_phi, 0.0);
    point.snap =
        radius * pow(omega, 4.0) * Eigen::Vector3d(cos_phi, sin_phi, 0.0);

    trajectory.points.push_back(point);
  }

  // Add last point at phi_end
  const double phi = phi_start + phi_total;
  const double cos_phi = cos(phi);
  const double sin_phi = sin(phi);
  vision_common::TrajectoryPoint point;
  point.time_from_start = ros::Duration(fabs(phi_total / omega));
  point.position = radius * Eigen::Vector3d(cos_phi, sin_phi, 0.0) + center;
  point.velocity = radius * omega * Eigen::Vector3d(-sin_phi, cos_phi, 0.0);
  point.acceleration =
      radius * pow(omega, 2.0) * Eigen::Vector3d(-cos_phi, -sin_phi, 0.0);
  point.jerk =
      radius * pow(omega, 3.0) * Eigen::Vector3d(sin_phi, -cos_phi, 0.0);
  point.snap =
      radius * pow(omega, 4.0) * Eigen::Vector3d(cos_phi, sin_phi, 0.0);

  trajectory.points.push_back(point);

  return trajectory;
}

vision_common::Trajectory computeVerticalCircleTrajectory(
    const Eigen::Vector3d center, const double orientation, const double radius,
    const double speed, const double phi_start, const double phi_end,
    const double sampling_frequency) {
  /*
   * We use a coordinate system with x to the front, y to the left, and z up.
   * Orientation is the angle by which the circle is rotated about the z-axis
   * If the orientation = 0.0 then the circle is in the x-z plane
   * When setting phi_start = 0.0 and orientation = 0.0, the start point will
   * be at (radius, 0, 0) + center
   * If phi_end > phi_start the circle is going in positive direction around
   * the y-axis rotated by orientation. Otherwise it is going in negative
   * direction
   */

  vision_common::Trajectory trajectory;
  trajectory.trajectory_type =
      vision_common::Trajectory::TrajectoryType::GENERAL;

  const double phi_total = phi_end - phi_start;
  const double direction = phi_total / fabs(phi_total);
  const double omega = direction * fabs(speed / radius);
  const double angle_step = fabs(omega / sampling_frequency);

  const Eigen::Quaterniond q_ori = Eigen::Quaterniond(
      Eigen::AngleAxisd(vision_common::wrapMinusPiToPi(orientation),
                        Eigen::Vector3d::UnitZ()));

  for (double d_phi = 0.0; d_phi < fabs(phi_total); d_phi += angle_step) {
    const double phi = phi_start + direction * d_phi;
    const double cos_phi = cos(phi);
    const double sin_phi = sin(phi);
    vision_common::TrajectoryPoint point;
    point.time_from_start = ros::Duration(fabs(d_phi / omega));
    point.position =
        q_ori * (radius * Eigen::Vector3d(cos_phi, 0.0, -sin_phi)) + center;
    point.velocity =
        q_ori * (radius * omega * Eigen::Vector3d(-sin_phi, 0.0, -cos_phi));
    point.acceleration = q_ori * (radius * pow(omega, 2.0) *
                                  Eigen::Vector3d(-cos_phi, 0.0, sin_phi));
    point.jerk = q_ori * (radius * pow(omega, 3.0) *
                          Eigen::Vector3d(sin_phi, 0.0, cos_phi));
    point.snap = q_ori * (radius * pow(omega, 4.0) *
                          Eigen::Vector3d(cos_phi, 0.0, -sin_phi));

    trajectory.points.push_back(point);
  }

  // Add last point at phi_end
  const double phi = phi_start + phi_total;
  const double cos_phi = cos(phi);
  const double sin_phi = sin(phi);
  vision_common::TrajectoryPoint point;
  point.time_from_start = ros::Duration(fabs(phi_total / omega));
  point.position =
      q_ori * (radius * Eigen::Vector3d(cos_phi, 0.0, -sin_phi)) + center;
  point.velocity =
      q_ori * (radius * omega * Eigen::Vector3d(-sin_phi, 0.0, -cos_phi));
  point.acceleration = q_ori * (radius * pow(omega, 2.0) *
                                Eigen::Vector3d(-cos_phi, 0.0, sin_phi));
  point.jerk = q_ori * (radius * pow(omega, 3.0) *
                        Eigen::Vector3d(sin_phi, 0.0, cos_phi));
  point.snap = q_ori * (radius * pow(omega, 4.0) *
                        Eigen::Vector3d(cos_phi, 0.0, -sin_phi));

  trajectory.points.push_back(point);

  return trajectory;
}

}  // namespace circles

}  // namespace trajectory_generation_helper
