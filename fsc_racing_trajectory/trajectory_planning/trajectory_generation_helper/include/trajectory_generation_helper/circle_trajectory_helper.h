#pragma once

#include <vision_common/trajectory.h>
#include <Eigen/Dense>

namespace trajectory_generation_helper {

namespace circles {

vision_common::Trajectory computeHorizontalCircleTrajectory(
    const Eigen::Vector3d center, const double radius, const double speed,
    const double phi_start, const double phi_end,
    const double sampling_frequency);

vision_common::Trajectory computeVerticalCircleTrajectory(
    const Eigen::Vector3d center, const double orientation, const double radius,
    const double speed, const double phi_start, const double phi_end,
    const double sampling_frequency);

}  // namespace circles

}  // namespace trajectory_generation_helper
