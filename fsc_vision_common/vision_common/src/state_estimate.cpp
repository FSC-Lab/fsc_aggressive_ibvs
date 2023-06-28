#include "vision_common/state_estimate.h"

namespace vision_common {

StateEstimate::StateEstimate()
    : timestamp(ros::Time::now()),
      coordinate_frame(CoordinateFrame::INVALID),
      position(Eigen::Vector3d::Zero()),
      velocity(Eigen::Vector3d::Zero()),
      orientation(Eigen::Quaterniond::Identity()),
      gates(),
      collective_thrust(0.0),
      bodyrates(Eigen::Vector3d::Zero()) {}

StateEstimate::StateEstimate(const vision_msgs::StateEstimate& msg) {
  timestamp = msg.header.stamp;

  coordinate_frame = CoordinateFrame::INVALID;
  if (msg.header.frame_id.compare("world") == 0) {
    coordinate_frame = CoordinateFrame::WORLD;
  } else if (msg.header.frame_id.compare("optitrack") == 0) {
    coordinate_frame = CoordinateFrame::OPTITRACK;
  } else if (msg.header.frame_id.compare("vision") == 0) {
    coordinate_frame = CoordinateFrame::VISION;
  } else if (msg.header.frame_id.compare("local") == 0) {
    coordinate_frame = CoordinateFrame::LOCAL;
  }

  position = geometryToEigen(msg.odometry.pose.pose.position);
  velocity = geometryToEigen(msg.odometry.twist.twist.linear);
  orientation = geometryToEigen(msg.odometry.pose.pose.orientation);
  bodyrates = geometryToEigen(msg.odometry.twist.twist.angular);

  collective_thrust = msg.collective_thrust;
  gates = vision_common::GateFeatureArray(msg.gates);

}

StateEstimate::StateEstimate(const nav_msgs::Odometry& msg) {
  timestamp = msg.header.stamp;

  coordinate_frame = CoordinateFrame::INVALID;
  if (msg.header.frame_id.compare("world") == 0) {
    coordinate_frame = CoordinateFrame::WORLD;
  } else if (msg.header.frame_id.compare("optitrack") == 0) {
    coordinate_frame = CoordinateFrame::OPTITRACK;
  } else if (msg.header.frame_id.compare("vision") == 0) {
    coordinate_frame = CoordinateFrame::VISION;
  } else if (msg.header.frame_id.compare("local") == 0) {
    coordinate_frame = CoordinateFrame::LOCAL;
  }

  position = geometryToEigen(msg.pose.pose.position);
  velocity = geometryToEigen(msg.twist.twist.linear);
  orientation = geometryToEigen(msg.pose.pose.orientation);
  bodyrates = geometryToEigen(msg.twist.twist.angular);

  collective_thrust = 0.0;
  gates.clear();
}

StateEstimate::~StateEstimate() {}

vision_msgs::StateEstimate StateEstimate::toRosMessage() const {
  vision_msgs::StateEstimate msg;

  msg.header.stamp = timestamp;
  msg.odometry.header.stamp = timestamp;

  switch (coordinate_frame) {
    case CoordinateFrame::WORLD:
      msg.header.frame_id = "world";
      msg.odometry.header.frame_id = "world";
      break;
    case CoordinateFrame::OPTITRACK:
      msg.header.frame_id = "optitrack";
      msg.odometry.header.frame_id = "optitrack";
      break;
    case CoordinateFrame::VISION:
      msg.header.frame_id = "vision";
      msg.odometry.header.frame_id = "vision";
      break;
    case CoordinateFrame::LOCAL:
      msg.header.frame_id = "local";
      msg.odometry.header.frame_id = "vision";
      break;
    default:
      msg.header.frame_id = "invalid";
      msg.odometry.header.frame_id = "vision";
      break;
  }

  msg.odometry.child_frame_id = "body";
  msg.odometry.pose.pose.position = vectorToPoint(eigenToGeometry(position));
  msg.odometry.twist.twist.linear = eigenToGeometry(velocity);
  msg.odometry.pose.pose.orientation = eigenToGeometry(orientation);

  msg.gates = gates.toRosMessage();

  msg.collective_thrust = collective_thrust;
  msg.odometry.twist.twist.angular = eigenToGeometry(bodyrates);

  return msg;
}

void StateEstimate::transformVelocityToWorldFrame() {
  velocity = orientation * velocity;
}

bool StateEstimate::isValid() const {
  // if (coordinate_frame == CoordinateFrame::INVALID) {
  //   return false;
  // }
  if (std::isnan(position.norm())) {
    return false;
  }
  if (std::isnan(velocity.norm())) {
    return false;
  }
  if (std::isnan(orientation.norm())) {
    return false;
  }
  if (std::isnan(bodyrates.norm())) {
    return false;
  }
  if (std::isnan(collective_thrust)) {
    return false;
  }

  return true;
}
}  // namespace vision_common