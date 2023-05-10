#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <Eigen/Dense>
#include <math.h>

namespace autopilot_monitor {

enum class FlightMode {
  GO_TO_GOAL,
  TRAJECTORY_TRACK,
  VELOCITY_TRACK,
  AUTO_RACE,
  FOLLOW,
  HOME,
  LAND
};

enum class ControlMode {
  NONE,
  PX4_POSITION_CTL,
  MPC_POSITION_CTL,
  MPC_VIEW_CTL,
  PX4_FEED_THROUGH
};

enum class MavState {
  UNINIT,
  BOOT,
  CALIBRATING,
  STANDBY,
  ACTIVE,
  CRITICAL,
  EMERGENCY,
  POWEROFF,
  FLIGHT_TERMINATION,
};

enum class Px4State {
  MANUAL,
  ACRO,
  ALTCTL,
  POSCTL,
  OFFBOARD,
  STABILIZED,
  RATTITUDE,
  AUTO_MISSION,
  AUTO_LOITER,
  AUTO_RTL,
  AUTO_LAND,
  AUTO_RTGS,
  AUTO_READY,
  AUTO_TAKEOFF,
  NONE,
};

struct ModelParameters {
  int param_mpc_auto_mode = -1;        // Auto sub-mode - 0: default line tracking, 1 jerk-limited trajectory
  float param_mpc_jerk_min = NAN;      // Velocity-based minimum jerk limit
  float param_mpc_jerk_max = NAN;      // Velocity-based maximum jerk limit
  float param_mpc_acc_up_max = NAN;    // Maximum vertical acceleration in velocity controlled modes upward
  float param_mpc_z_vel_max_up = NAN;  // Maximum vertical ascent velocity
  float param_mpc_acc_down_max = NAN;  // Maximum vertical acceleration in velocity controlled modes down
  float param_mpc_z_vel_max_dn = NAN;  // Maximum vertical descent velocity
  float param_mpc_acc_hor = NAN;       // Maximum horizontal acceleration for auto mode and
                                       // maximum deceleration for manual mode
  float param_mpc_xy_cruise = NAN;     // Desired horizontal velocity in mission
  float param_mpc_tko_speed = NAN;     // Takeoff climb rate
  float param_mpc_land_speed = NAN;    // Landing descend rate
  float param_mpc_yawrauto_max = NAN;

  float param_nav_acc_rad = NAN;

  // TODO: add estimator limitations for max speed and height

  float param_cp_dist = NAN;  // Collision Prevention distance to keep from obstacle. -1 for disabled
};

inline geometry_msgs::Point toPoint(const Eigen::Vector3d& ev3) {
  geometry_msgs::Point gmp;
  gmp.x = ev3.x();
  gmp.y = ev3.y();
  gmp.z = ev3.z();
  return gmp;
}

inline geometry_msgs::Vector3 toVector3(const Eigen::Vector3d& ev3) {
  geometry_msgs::Vector3 gmv3;
  gmv3.x = ev3.x();
  gmv3.y = ev3.y();
  gmv3.z = ev3.z();
  return gmv3;
}

inline geometry_msgs::Quaternion toQuaternion(const Eigen::Quaterniond& eq) {
  geometry_msgs::Quaternion q;
  q.x = eq.x();
  q.y = eq.y();
  q.z = eq.z();
  q.w = eq.w();
  return q;
}

inline geometry_msgs::Twist toTwist(const Eigen::Vector3d& l,
                                    const Eigen::Vector3d& a) {
  geometry_msgs::Twist gmt;
  gmt.linear = toVector3(l);
  gmt.angular = toVector3(a);
  return gmt;
}

inline geometry_msgs::PoseStamped toPoseStamped(const Eigen::Vector3d& ev3,
                                                const Eigen::Quaterniond& eq) {
  geometry_msgs::PoseStamped gmps;
  gmps.header.stamp = ros::Time::now();
  gmps.header.frame_id = "/world";
  gmps.pose.position = toPoint(ev3);
  gmps.pose.orientation = toQuaternion(eq);
  return gmps;
}

} // namespace offb_basic