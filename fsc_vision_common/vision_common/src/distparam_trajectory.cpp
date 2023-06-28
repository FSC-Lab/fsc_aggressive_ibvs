#include <vision_common/distparam_trajectory.h>

namespace vision_common {

void DistparamTrajectory::plan(const Eigen::Vector3d& focus,
                               const vision_common::Trajectory& trajectory) {
  if (trajectory.points.size() < 2) {
    std::cout << "DistparamTrajectory: Less than 2 points." << std::endl;
    return;
  }

  std::vector<Eigen::Vector3d> points;
  std::vector<double> s_vec;

  auto p0 = trajectory.points.begin();
  auto p_last = trajectory.points.end();
  p_last--;

  double d = distance(p0->position, focus);
  if (d > max_distance_) {
    std::cout << "First distance is larger than the maximum distance."
              << std::endl;
    return;
  }

  points.push_back(p0->position);
  s_vec.push_back(toS(d));
  p0++;
  double d_last = d;

  for (; p0 != trajectory.points.end(); ++p0) {
    d = distance(p0->position, focus);
    if (d > d_last) {
      break;
    }

    if (fabs(d - d_last) >= gap_ || p0 == p_last) {
      points.push_back(p0->position);
      s_vec.push_back(toS(d));
      d_last = d;
    }
  }

  spline3d_.plan(points, s_vec);
}

}  // namespace vision_common