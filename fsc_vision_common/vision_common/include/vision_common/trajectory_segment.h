#pragma once

#include <list>

#include <ros/time.h>
#include <vision_msgs/TrajectorySegment.h>
#include "vision_common/trajectory_point.h"

namespace vision_common {

using std::string;

struct TrajectorySegment {
  TrajectorySegment();

  TrajectorySegment(const vision_msgs::TrajectorySegment& msg);

  virtual ~TrajectorySegment();

  vision_common::TrajectoryPoint getStateAtTime(
      const ros::Duration& time_from_start);

  vision_common::TrajectoryPoint getStateAtLength(
      const double length_from_start);

  vision_common::TrajectoryPoint getStateAtDistance(
      const double distance_to_gate);

  vision_msgs::TrajectorySegment toRosMessage() const;

  inline ros::Duration getTimeSegmentStart() const {
    return points.front().time_from_start;
  };
  inline ros::Duration getTimeSegmentEnd() const {
    return points.back().time_from_start;
  };

  inline double getLengthSegmentStart() const {
    return points.front().length_from_start;
  };
  inline double getLengthSegmentEnd() const {
    return points.back().length_from_start;
  };

  inline double getDistanceSegmentStart() const {
    return points.front().distance_to_gate;
  };
  inline double getDistanceSegmentEnd() const {
    return points.back().distance_to_gate;
  };


  int segment_id;
  string gate_id;
  string next_id;

  // timestamp in the whole trajectory
  ros::Duration time_segment_start;
  ros::Duration time_segment_end;

  // arc length (progress) in the whole trajectory
  double length_segment_start;
  double length_segment_end;

  // distance to the gate
  double distance_segment_start;
  double distance_segment_end;

  std::list<vision_common::TrajectoryPoint> points;
};


}  // namespace vision_common
