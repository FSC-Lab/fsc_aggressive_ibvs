#pragma once

// ROS-free plain data types for mpc_core.
//
// These replace the vision_common message-backed structs (StateEstimate,
// Trajectory, TrajectoryPoint, ControlCommand) that coupled the original
// controller to ROS1. The numerical library operates purely on these PODs; the
// ROS2 node layer converts to/from ROS messages at its boundary.
//
// Timestamps are plain seconds (double). The node stamps ROS messages; the core
// only needs monotonically increasing times spaced by the MPC timestep.

#include <cmath>
#include <vector>

#include <Eigen/Eigen>

namespace mpc_core {

// Yaw (ZYX / heading) extracted from a quaternion. ROS-free replacement for
// vision_common::quaternionToEulerAnglesZYX(...).z().
inline double yawFromQuaternion(const Eigen::Quaterniond& q) {
  const double siny_cosp = 2.0 * (q.w() * q.z() + q.x() * q.y());
  const double cosy_cosp = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
  return std::atan2(siny_cosp, cosy_cosp);
}

// Odometry + line-of-sight measurement fed into the MPC.
struct StateEstimate {
  double timestamp{0.0};
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};

  // Bearing (line-of-sight to the target) as a unit quaternion f1, and the
  // range r1 to the target [m]. In the ROS1 code these lived in
  // gates.gates[0].center.{normal_vector.orientation, distance}.
  Eigen::Quaterniond bearing{Eigen::Quaterniond::Identity()};
  double distance{10.0};

  bool bearing_valid{false};  // set by the core after a norm check
};

// One reference sample of the desired flight trajectory.
struct TrajectoryPoint {
  double time_from_start{0.0};  // seconds from trajectory start
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  double distance_rate{0.0};    // desired closing rate toward the target
};

struct Trajectory {
  std::vector<TrajectoryPoint> points;
};

// Output of the velocity-body + pitch-gimbal MPC. Maps directly to a PX4
// offboard TrajectorySetpoint (velocity + yawspeed) plus a gimbal setpoint.
struct ControlCommand {
  double timestamp{0.0};
  double expected_execution_time{0.0};
  bool armed{false};

  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};  // commanded world velocity
  double yaw_rate{0.0};                                // camera pan
  double gimbal_angle{0.0};                            // camera tilt setpoint
  std::vector<double> slack_variables;

  void zero() {
    armed = false;
    velocity.setZero();
    yaw_rate = 0.0;
    gimbal_angle = 0.0;
    slack_variables.clear();
  }
};

// Cubic-polynomial reference segment (per axis: a*s^3 + b*s^2 + c*s + y),
// ROS-free replacement for vision_common::PolynomialTrajectory. Each Vector4d
// holds (a, b, c, y); distance_start is the segment's start abscissa (x_s).
struct PolynomialTrajectory {
  Eigen::Vector4d x{Eigen::Vector4d::Zero()};
  Eigen::Vector4d y{Eigen::Vector4d::Zero()};
  Eigen::Vector4d z{Eigen::Vector4d::Zero()};
  double distance_start{0.0};
};

}  // namespace mpc_core
