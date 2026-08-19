#pragma once

// Frame conversions between the ROS convention (world ENU, body FLU) used by
// mpc_controller_node and the PX4 convention (world NED, body FRD) used on the
// uXRCE-DDS bus. Kept header-only and dependency-free (Eigen only) so the frame
// math is isolated in exactly one place.
//
// References: PX4 / px4_ros_com frame_transforms.
//   - world  NED <-> ENU : rotation of pi about (1,1,0)/sqrt(2)  (involutive)
//   - body   FRD <-> FLU : rotation of pi about X                (involutive)

#include <Eigen/Eigen>

namespace px4_bridge {

// World-frame swap. R = [[0,1,0],[1,0,0],[0,0,-1]] is its own inverse, so the
// same function maps ENU->NED and NED->ENU:  (x,y,z) -> (y,x,-z).
inline Eigen::Vector3d swapEnuNed(const Eigen::Vector3d& v) {
  return Eigen::Vector3d(v.y(), v.x(), -v.z());
}

// Body-frame swap FRD<->FLU:  (x,y,z) -> (x,-y,-z). Also involutive.
inline Eigen::Vector3d swapFrdFlu(const Eigen::Vector3d& v) {
  return Eigen::Vector3d(v.x(), -v.y(), -v.z());
}

// Static rotation quaternions (w, x, y, z).
//   q(NED->ENU): pi about (1,1,0)/sqrt(2)
//   q(FRD->FLU): pi about X
inline const Eigen::Quaterniond& qNedEnu() {
  static const Eigen::Quaterniond q(0.0, M_SQRT1_2, M_SQRT1_2, 0.0);
  return q;
}
inline const Eigen::Quaterniond& qFrdFlu() {
  static const Eigen::Quaterniond q(0.0, 1.0, 0.0, 0.0);
  return q;
}

// Attitude: body FRD w.r.t. world NED  ->  body FLU w.r.t. world ENU.
//   q_enu_flu = q(NED->ENU) * q_ned_frd * q(FRD->FLU)
inline Eigen::Quaterniond attitudeNedToEnu(const Eigen::Quaterniond& q_ned_frd) {
  return (qNedEnu() * q_ned_frd * qFrdFlu()).normalized();
}

// Yaw: ENU yaw (about +Up, CCW) <-> NED yaw (about +Down, CW).  psi_ned = pi/2 - psi_enu.
inline double yawEnuToNed(double yaw_enu) { return M_PI_2 - yaw_enu; }
// Yaw rate flips sign between the two conventions (opposite rotation axis).
inline double yawRateEnuToNed(double yaw_rate_enu) { return -yaw_rate_enu; }

}  // namespace px4_bridge
