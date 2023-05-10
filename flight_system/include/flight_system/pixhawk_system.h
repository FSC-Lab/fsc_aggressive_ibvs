#pragma once

#include <flight_system/filght_system.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CompanionProcessStatus.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/Param.h>
#include <mavros_msgs/ParamGet.h>
#include <vision_common/parameter_helper.h>
#include <vision_common/math_common.h>
#include <vision_common/conversion_common.h>
#include <std_msgs/Empty.h>
#include <mutex>

namespace fs {

struct PixhawkParameters {
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
  float param_cp_dist = NAN;  // Collision Prevention distance to keep from obstacle. -1 for disabled
};


class PixhawkSystem : public FlightSystem {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  enum class PixhawkMode {
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

  PixhawkSystem();

  PixhawkSystem(
      const double max_thrust,
      const double min_thrust,
      const double thrust_ratio = 1.0);

  ~PixhawkSystem();

  void configTopics();

  void publishTopics();

  void sendBodyRates(const vision_common::ControlCommand& command);

  void sendPosition(
    const Eigen::Vector3d& position,
    const Eigen::Quaterniond& orientation);

  bool ready();

 private:
  void pixhawkModeCallback(const mavros_msgs::State& msg);

  void pixhawkParamsCallback(const mavros_msgs::Param& msg);

  void optitrackCallback(const geometry_msgs::PoseStamped& msg);

  double normalizeThrust(const double thrust, const double max_thrust,
                         const double min_thrust);


  bool loadParameters();

 private:
  ros::Subscriber pixhawk_mode_sub_;
  ros::Subscriber pixhawk_param_sub_;
  ros::Subscriber optitrack_sub_;
  
  PixhawkMode pixhawk_mode_;
  PixhawkParameters pixhawk_params_;
  bool offboard_triggered_;
  bool motion_capture_ready_;
  ros::Time time_last_motion_capture_received_;
  double motion_capture_timeout_;
  bool use_motion_capture_;
};

}  // namespace fs
