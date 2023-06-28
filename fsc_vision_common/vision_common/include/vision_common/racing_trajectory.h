#pragma once

#include <list>

#include <ros/time.h>
#include <vision_msgs/RacingTrajectory.h>
#include "vision_common/trajectory_segment.h"

namespace vision_common {

struct RacingTrajectory {
  RacingTrajectory();

  RacingTrajectory(const vision_common::TrajectorySegment& segment);

  RacingTrajectory(const vision_msgs::RacingTrajectory& msg);

  virtual ~RacingTrajectory();

  vision_common::TrajectoryPoint getStateAtTime(
      const ros::Duration& time_from_start);

  vision_common::TrajectoryPoint getStateAtLength(
      const double length_from_start);

  vision_common::TrajectoryPoint getStateAtDistance(
      const string gate_id, const double distance_to_gate);

  vision_common::TrajectorySegment getSegmentAtGate(const string gate_id);

  vision_msgs::RacingTrajectory toRosMessage() const;

  ros::Time timestamp;
  std::list<TrajectorySegment> segments;
};

}  // namespace vision_common