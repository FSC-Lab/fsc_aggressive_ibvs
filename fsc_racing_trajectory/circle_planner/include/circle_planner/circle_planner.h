#pragma once

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>

#include <vision_common/conversion_common.h>
#include <vision_common/math_common.h>
#include <vision_common/parameter_helper.h>
#include <vision_common/state_estimate.h>
#include <vision_common/trajectory.h>
#include <vision_common/trajectory_point.h>

#include <vision_msgs/StateEstimate.h>
#include <vision_msgs/Trajectory.h>
#include <vision_msgs/TrajectoryPoint.h>
#include <vision_msgs/TuneData48.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <trajectory_generation_helper/circle_trajectory_helper.h>
#include <trajectory_generation_helper/heading_trajectory_helper.h>
#include <trajectory_generation_helper/polynomial_trajectory_helper.h>

#include <mutex>
#include <string>
#include <thread>

namespace fsc {

class CirclePlanner {
 public:
  CirclePlanner();
  ~CirclePlanner();

  void run();
  void plan();
 private:
  void odometryCallback(const nav_msgs::Odometry& msg);
  void paramsCallback(const vision_msgs::TuneData48& msg);
  void clickedGoalCallback(const geometry_msgs::PoseStamped& msg);
  void testHorizontalCircle();
  bool loadParameters();

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber odom_sub_;
  ros::Subscriber params_sub_;
  ros::Subscriber clicked_goal_sub_;
  ros::Publisher traj_pub_;

  std::mutex mutex_;
  vision_common::StateEstimate state_est_;
  bool velocity_estimate_in_world_frame_ = false;

  Eigen::Vector3d position_goal_;
  bool ready_ = false;

  int order_ = 5;
  double max_vel_ = 3.0;
  double max_thrust_ = 20.0;
  double max_roll_pitch_rate_ = 1.0;
  static constexpr double kExecLoopRate_ = 20.0;

  Eigen::Vector3d hcircle_center_;
  double hradius_;
  double hphi_start_;
  double hphi_end_;
  double hnum_circles_;


};

}