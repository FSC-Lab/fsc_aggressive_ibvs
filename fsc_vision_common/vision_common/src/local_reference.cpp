#include "vision_common/local_reference.h"

namespace vision_common {

LocalReference::LocalReference()
    : timestamp(ros::Time::now()),
      feature_meas(),
      feature_pose(),
      gate(),
      reference_trajectory(),
      flight_mission(FlightMission::UNDEFINED),
      replan_triggered(false) {}

LocalReference::LocalReference(
    const vision_msgs::LocalReference& msg) {
  timestamp = msg.header.stamp;

  feature_meas = vision_common::VisualFeature(msg.feature_meas);
  feature_pose = vision_common::StateEstimate(msg.feature_pose);
  gate = vision_common::GateFeature(msg.gate);
  reference_trajectory = vision_common::Trajectory(msg.reference_trajectory);
  replan_triggered = msg.replan_triggered;

  switch (msg.tracking_feedback.tracking_state) {
    case msg.tracking_feedback.TAKE_OFF:
      flight_mission = FlightMission::TAKE_OFF;
      break;
    case msg.tracking_feedback.LAND:
      flight_mission = FlightMission::LAND;
      break;
    case msg.tracking_feedback.HOVER:
      flight_mission = FlightMission::HOVER;
      break;
    case msg.tracking_feedback.POSE_LOCK:
      flight_mission = FlightMission::POSE_LOCK;
      break;
    case msg.tracking_feedback.TRAJECTORY_TRACK:
      flight_mission = FlightMission::TRAJECTORY_TRACK;
      break;
    case msg.tracking_feedback.POSITION_TRACK:
      flight_mission = FlightMission::POSITION_TRACK;
      break;
    default:
      flight_mission = FlightMission::UNDEFINED;
      break;
  }
}

LocalReference::~LocalReference() {}

vision_msgs::LocalReference LocalReference::toRosMessage() const {
  vision_msgs::LocalReference ros_msg;
  ros_msg.header.stamp = timestamp;

  ros_msg.feature_meas = feature_meas.toRosMessage();
  ros_msg.feature_pose = feature_pose.toRosMessage();
  ros_msg.gate = gate.toRosMessage();
  ros_msg.reference_trajectory = reference_trajectory.toRosMessage();
  ros_msg.replan_triggered = replan_triggered;

  switch (flight_mission) {
    case FlightMission::TAKE_OFF:
      ros_msg.tracking_feedback.tracking_state = ros_msg.tracking_feedback.TAKE_OFF;
      break;
    case FlightMission::LAND:
      ros_msg.tracking_feedback.tracking_state = ros_msg.tracking_feedback.TAKE_OFF;
      break;
    case FlightMission::HOVER:
      ros_msg.tracking_feedback.tracking_state = ros_msg.tracking_feedback.HOVER;
      break;
    case FlightMission::POSE_LOCK:
      ros_msg.tracking_feedback.tracking_state = ros_msg.tracking_feedback.POSE_LOCK;
      break;
    case FlightMission::TRAJECTORY_TRACK:
      ros_msg.tracking_feedback.tracking_state = ros_msg.tracking_feedback.TRAJECTORY_TRACK;
      break;
    case FlightMission::POSITION_TRACK:
      ros_msg.tracking_feedback.tracking_state = ros_msg.tracking_feedback.POSITION_TRACK;
      break;
  }

  return ros_msg;
}

}