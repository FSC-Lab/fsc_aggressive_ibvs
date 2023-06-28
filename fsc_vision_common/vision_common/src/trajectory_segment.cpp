#include "vision_common/trajectory_segment.h"
#include "vision_common/math_common.h"

namespace vision_common {

TrajectorySegment::TrajectorySegment()
    : segment_id(-1),
      gate_id(""),
      next_id(""),
      time_segment_start(ros::Duration(0.0)),
      time_segment_end(ros::Duration(0.0)),
      length_segment_start(0.0),
      length_segment_end(0.0),
      distance_segment_start(0.0),
      distance_segment_end(0.0),
      points() {}

TrajectorySegment::TrajectorySegment(
    const vision_msgs::TrajectorySegment& msg) {
  segment_id = msg.segment_id;
  gate_id = msg.gate_id;
  next_id = msg.next_id;

  // time_segment_start = msg.time_segment_start;
  // time_segment_end = msg.time_segment_end;

  // length_segment_start = msg.length_segment_start;
  // length_segment_end = msg.length_segment_end;

  // distance_segment_start = msg.distance_segment_start;
  // distance_segment_end = msg.distance_segment_end;

  for (int i = 0; i < msg.points.size(); i++) {
    points.push_back(vision_common::TrajectoryPoint(msg.points[i]));
  }
}

TrajectorySegment::~TrajectorySegment() {}

vision_common::TrajectoryPoint TrajectorySegment::getStateAtTime(
    const ros::Duration& time_from_start) {
  if (time_from_start <= points.front().time_from_start) {
    return points.front();
  }
  if (time_from_start >= points.back().time_from_start) {
    return points.back();
  }

  vision_common::TrajectoryPoint trajectory_point;

  // Find points p0 and p1 such that
  // p0.time_from_start <= time_from_start <= p1.time_from_start
  std::list<vision_common::TrajectoryPoint>::const_iterator p1;
  for (p1 = points.begin(); p1 != points.end(); p1++) {
    if (p1->time_from_start > time_from_start) {
      break;
    }
  }
  std::list<vision_common::TrajectoryPoint>::const_iterator p0 = std::prev(p1);
  const double interp_ratio =
      (time_from_start - p0->time_from_start).toSec() /
      (p1->time_from_start - p0->time_from_start).toSec();

  return interpolate(*p0, *p1, interp_ratio);
}

vision_common::TrajectoryPoint TrajectorySegment::getStateAtLength(
    const double length_from_start) {
  if (length_from_start <= points.front().length_from_start) {
    return points.front();
  }
  if (length_from_start >= points.back().length_from_start) {
    return points.back();
  }

  vision_common::TrajectoryPoint trajectory_point;

  // Find points p0 and p1 such that
  // p0.length_from_start <= length_from_start <= p1.length_from_start
  std::list<vision_common::TrajectoryPoint>::const_iterator p1;
  for (p1 = points.begin(); p1 != points.end(); p1++) {
    if (p1->length_from_start > length_from_start) {
      break;
    }
  }
  std::list<vision_common::TrajectoryPoint>::const_iterator p0 = std::prev(p1);
  const double interp_ratio =
      (length_from_start - p0->length_from_start) /
      (p1->length_from_start - p0->length_from_start);

  return interpolate(*p0, *p1, interp_ratio);
}

vision_common::TrajectoryPoint TrajectorySegment::getStateAtDistance(
    const double distance_to_gate) {
  // point is farther than the first point
  if (distance_to_gate >= points.front().distance_to_gate) {
    return points.front();
  }
  // point is closer than the last point
  if (distance_to_gate <= points.back().distance_to_gate) {
    return points.back();
  }

  vision_common::TrajectoryPoint trajectory_point;

  // Find points p0 and p1 such that
  // p0.distance_to_gate >= length_from_start >= p1.distance_to_gate
  std::list<vision_common::TrajectoryPoint>::const_iterator p1;
  for (p1 = points.begin(); p1 != points.end(); p1++) {
    if (p1->distance_to_gate < distance_to_gate) {
      break;
    }
  }
  std::list<vision_common::TrajectoryPoint>::const_iterator p0 = std::prev(p1);
  const double interp_ratio =
      (p0->distance_to_gate - distance_to_gate) /
      (p0->distance_to_gate - p1->distance_to_gate);

  return interpolate(*p0, *p1, interp_ratio);
}

vision_msgs::TrajectorySegment TrajectorySegment::toRosMessage() const {
  vision_msgs::TrajectorySegment ros_msg;

  ros_msg.segment_id = segment_id;
  ros_msg.gate_id = gate_id;
  ros_msg.next_id = next_id;

  // ros_msg.time_segment_start = time_segment_start;
  // ros_msg.time_segment_end = time_segment_end;

  // ros_msg.length_segment_start = length_segment_start;
  // ros_msg.length_segment_end = length_segment_end;

  // ros_msg.distance_segment_start = distance_segment_start;
  // ros_msg.distance_segment_end = distance_segment_end;

  std::list<vision_common::TrajectoryPoint>::const_iterator it;
  for (it = points.begin(); it != points.end(); it++) {
    ros_msg.points.push_back(it->toRosMessage());
  }

  return ros_msg;
}

}  // namespace vision_common