#include "vision_common/trajectory_point.h"

namespace vision_common {

TrajectoryPoint::TrajectoryPoint()
    : time_from_start(ros::Duration(0.0)),
      position(Eigen::Vector3d::Zero()),
      orientation(Eigen::Quaterniond::Identity()),
      velocity(Eigen::Vector3d::Zero()),
      acceleration(Eigen::Vector3d::Zero()),
      jerk(Eigen::Vector3d::Zero()),
      snap(Eigen::Vector3d::Zero()),
      bodyrates(Eigen::Vector3d::Zero()),
      angular_acceleration(Eigen::Vector3d::Zero()),
      angular_jerk(Eigen::Vector3d::Zero()),
      angular_snap(Eigen::Vector3d::Zero()),
      heading(0.0),
      heading_rate(0.0),
      heading_acceleration(0.0),
      length_from_start(0),
      tangent(Eigen::Vector3d::Zero()),
      distance_to_gate(0.0),
      distance_rate(0.0),
      gate_id("") {}

TrajectoryPoint::TrajectoryPoint(
    const vision_msgs::TrajectoryPoint& msg) {
  time_from_start = msg.time_from_start;
  length_from_start = msg.length_from_start;
  tangent = geometryToEigen(msg.tangent);
  distance_to_gate = msg.distance_to_gate;
  gate_id = msg.gate_id;

  position = geometryToEigen(msg.pose.position);
  orientation = geometryToEigen(msg.pose.orientation);

  velocity = geometryToEigen(msg.velocity.linear);
  acceleration = geometryToEigen(msg.acceleration.linear);
  jerk = geometryToEigen(msg.jerk.linear);
  snap = geometryToEigen(msg.snap.linear);

  bodyrates = geometryToEigen(msg.velocity.angular);
  angular_acceleration =
      geometryToEigen(msg.acceleration.angular);
  angular_jerk = geometryToEigen(msg.jerk.angular);
  angular_snap = geometryToEigen(msg.snap.angular);

  heading = msg.heading;
  heading_rate = msg.heading_rate;
  heading_acceleration = msg.heading_acceleration;
}

TrajectoryPoint::~TrajectoryPoint() {}

vision_msgs::TrajectoryPoint TrajectoryPoint::toRosMessage() const {
  vision_msgs::TrajectoryPoint ros_msg;

  ros_msg.time_from_start = time_from_start;
  ros_msg.length_from_start = length_from_start;
  ros_msg.tangent = eigenToGeometry(tangent);
  ros_msg.distance_to_gate = distance_to_gate;
  ros_msg.gate_id = gate_id;
  
  ros_msg.pose.position = vectorToPoint(eigenToGeometry(position));
  ros_msg.pose.orientation = eigenToGeometry(orientation);

  ros_msg.velocity.linear = eigenToGeometry(velocity);
  ros_msg.acceleration.linear = eigenToGeometry(acceleration);
  ros_msg.jerk.linear = eigenToGeometry(jerk);
  ros_msg.snap.linear = eigenToGeometry(snap);

  ros_msg.velocity.angular = eigenToGeometry(bodyrates);
  ros_msg.acceleration.angular = eigenToGeometry(angular_acceleration);
  ros_msg.jerk.angular = eigenToGeometry(angular_jerk);
  ros_msg.snap.angular = eigenToGeometry(angular_snap);

  ros_msg.heading = heading;
  ros_msg.heading_rate = heading_rate;
  ros_msg.heading_acceleration = heading_acceleration;

  return ros_msg;
}

}  // namespace vision_common
