#pragma once

#include <ros/duration.h>
#include <Eigen/Dense>

#include "vision_common/visual_feature.h"
#include "vision_common/state_estimate.h"
#include "vision_common/gate_feature.h"
#include "vision_common/trajectory.h"

#include "vision_msgs/LocalReference.h"

namespace vision_common {

struct LocalReference {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  enum class FlightMission {
    TAKE_OFF = 0,
    LAND = 1,
    HOVER = 2,
    POSE_LOCK = 3,
    TRAJECTORY_TRACK = 4,
    POSITION_TRACK = 5,
    UNDEFINED = 99
  };

  LocalReference();
  LocalReference(const vision_msgs::LocalReference& msg);
  virtual ~LocalReference();

  vision_msgs::LocalReference toRosMessage() const;

  ros::Time timestamp;
  vision_common::VisualFeature feature_meas;
  vision_common::StateEstimate feature_pose;
  vision_common::GateFeature gate;
  vision_common::Trajectory reference_trajectory;
  FlightMission flight_mission;
  bool replan_triggered;
};

}  // namespace vision_common
