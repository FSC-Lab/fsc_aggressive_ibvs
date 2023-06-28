#pragma once

#include <vision_common/cubic_spline3d.h>
#include <vision_common/trajectory.h>
#include <Eigen/Dense>

namespace vision_common {

class DistparamTrajectory {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  DistparamTrajectory() : DistparamTrajectory(100.0, 0.5) {}

  DistparamTrajectory(const double max_distance, const double gap)
      : max_distance_(max_distance), gap_(gap) {}

  ~DistparamTrajectory() {}

  void plan(const Eigen::Vector3d& focus,
            const vision_common::Trajectory& trajectory);

  inline double distance(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1) {
    return (p1 - p0).norm();
  }

  inline double toS(const double distance) const {
    return (max_distance_ - distance);
  }

  inline double fromS(const double s) const { return (max_distance_ - s); }

  inline Eigen::Vector3d getPoint(const double distance) {
    if (!isValid()) {
      return Eigen::Vector3d(0,0,0);
    }
    return spline3d_.position(toS(distance));
  }

  inline vision_common::PolynomialTrajectory getCoeffs(const double distance) {
    return spline3d_.coeffs(toS(distance));
  }

  inline bool isValid() const { return spline3d_.isValid(); }

  inline bool isInPath(const double distance) {
    return spline3d_.isInPath(toS(distance));
  }

 private:
  vision_common::CubicSpline3D spline3d_;
  double max_distance_;
  double gap_;
};

}  // namespace vision_common