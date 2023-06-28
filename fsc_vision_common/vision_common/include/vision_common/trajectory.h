#pragma once

#include <list>

#include <nav_msgs/Path.h>
#include <ros/time.h>
#include <vision_msgs/Trajectory.h>
#include "vision_common/trajectory_point.h"

namespace vision_common {
     

struct Trajectory {
  Trajectory();
  Trajectory(const vision_msgs::Trajectory& trajectory_msg);
  Trajectory(const vision_common::TrajectoryPoint& point);
  virtual ~Trajectory();

  vision_msgs::Trajectory toRosMessage() const;
  nav_msgs::Path toRosPath() const;
  vision_common::TrajectoryPoint getStateAtTime(
      const ros::Duration& time_from_start) const;

  ros::Time timestamp;

  enum class TrajectoryType {
    UNDEFINED,
    GENERAL,
    ACCELERATION,
    JERK,
    SNAP
  } trajectory_type;

  std::list<vision_common::TrajectoryPoint> points;
};

}  // namespace vision_common
