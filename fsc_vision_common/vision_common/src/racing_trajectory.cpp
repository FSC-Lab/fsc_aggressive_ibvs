#include "vision_common/racing_trajectory.h"

namespace vision_common {

RacingTrajectory::RacingTrajectory()
    : timestamp(ros::Time::now()), segments() {}

RacingTrajectory::RacingTrajectory(
    const vision_common::TrajectorySegment& segment)
    : timestamp(ros::Time::now()), segments() {
  segments.push_back(segment);
}

RacingTrajectory::RacingTrajectory(const vision_msgs::RacingTrajectory& msg) {
  timestamp = msg.header.stamp;

  for (int i = 0; i < msg.segments.size(); i++) {
    segments.push_back(vision_common::TrajectorySegment(msg.segments[i]));
  }
}

RacingTrajectory::~RacingTrajectory() {}

vision_common::TrajectoryPoint RacingTrajectory::getStateAtTime(
    const ros::Duration& time_from_start) {
  if (time_from_start <= segments.front().getTimeSegmentStart()) {
    return segments.front().points.front();
  }
  if (time_from_start >= segments.back().getTimeSegmentEnd()) {
    return segments.back().points.back();
  }

  auto ts = segments.begin();
  for (; ts != segments.end(); ts++) {
    if (ts->getTimeSegmentEnd() > time_from_start) {
      break;
    }
  }

  return ts->getStateAtTime(time_from_start);
}

vision_common::TrajectoryPoint RacingTrajectory::getStateAtLength(
    const double length_from_start) {
  if (length_from_start <= segments.front().getLengthSegmentStart()) {
    return segments.front().points.front();
  }
  if (length_from_start >= segments.back().getLengthSegmentEnd()) {
    return segments.back().points.back();
  }

  auto ts = segments.begin();
  for (; ts != segments.end(); ts++) {
    if (ts->getLengthSegmentEnd() > length_from_start) {
      break;
    }
  }

  return ts->getStateAtLength(length_from_start);
}

vision_common::TrajectoryPoint RacingTrajectory::getStateAtDistance(
    const string gate_id, const double distance_to_gate) {
  vision_common::TrajectorySegment segment = getSegmentAtGate(gate_id);
  if (segment.points.empty()) {
    return vision_common::TrajectoryPoint();
  }

  return segment.getStateAtDistance(distance_to_gate);
}

vision_common::TrajectorySegment RacingTrajectory::getSegmentAtGate(const string gate_id) {
  bool found = false;
  auto search = segments.begin();
  for (; search != segments.end(); ++search) {
    if (search->gate_id == gate_id) {
      found = true;
      break;
    }
  }

  if (!found) {
    return vision_common::TrajectorySegment();
  }

  return *search;
}

vision_msgs::RacingTrajectory RacingTrajectory::toRosMessage() const {
  vision_msgs::RacingTrajectory ros_msg;

  ros_msg.header.stamp = timestamp;

  std::list<vision_common::TrajectorySegment>::const_iterator it;
  for (it = segments.begin(); it != segments.end(); it++) {
    ros_msg.segments.push_back(it->toRosMessage());
  }

  return ros_msg;
}

}  // namespace vision_common