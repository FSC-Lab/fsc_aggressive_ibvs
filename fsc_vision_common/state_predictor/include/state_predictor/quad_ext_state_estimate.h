#pragma once

#include <vision_common/state_estimate.h>

namespace state_predictor
{

struct QuadExtStateEstimate : vision_common::StateEstimate
{
  QuadExtStateEstimate();
  QuadExtStateEstimate(const nav_msgs::Odometry& quad_state_est);
  QuadExtStateEstimate(
      const vision_common::StateEstimate& quad_state_est);
  virtual ~QuadExtStateEstimate();

  vision_common::StateEstimate getQuadStateEstimate();

  double thrust;

  // Constants
  static constexpr double kDefaultThrust = 0.0;
};

} // namespace state_predictor
