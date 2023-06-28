#include "circle_planner/circle_planner.h"

namespace fsc {
CirclePlanner::CirclePlanner()
  : nh_(ros::NodeHandle()),
    pnh_(ros::NodeHandle("~")),
    state_est_() {

  if (!loadParameters()) {
    ROS_ERROR("Fail to load CirclePlanner parameters!");
    return;
  }

  odom_sub_ = nh_.subscribe("odom_topic", 1, &CirclePlanner::odometryCallback, this);

  params_sub_ = nh_.subscribe("autopilot/tune_data_48", 1,
                              &CirclePlanner::paramsCallback, this);

  clicked_goal_sub_ = nh_.subscribe(
      "/move_base_simple/goal", 1, &CirclePlanner::clickedGoalCallback, this);

  traj_pub_ = nh_.advertise<vision_msgs::Trajectory>("autopilot/reference_trajectory", 10);

  position_goal_ = Eigen::Vector3d(0,0,3);



}

CirclePlanner::~CirclePlanner() {}

void CirclePlanner::run() {
  int counter = 0;
  ros::Rate command_rate(1.0);
  while (ros::ok() && counter<hnum_circles_) {
    // plan();
    counter++;
    testHorizontalCircle();
    ros::spinOnce();
    command_rate.sleep();
  }
}

void CirclePlanner::plan() {
  if (!ready_) {
    ROS_WARN("Odometry Unavailable.");
    return;
  }

  if ((position_goal_ - state_est_.position).norm() < 0.2) {
    ROS_INFO("Goal Reached.");
    return;
  }

  ROS_INFO("Planning...");

  vision_common::TrajectoryPoint start_state;
  start_state.position = state_est_.position;
  // start_state.position = Eigen::Vector3d(0,0,1);
  start_state.heading = vision_common::wrapMinusPiToPi(0.0);
  vision_common::TrajectoryPoint end_state;
  end_state.position = position_goal_;
  end_state.heading = vision_common::wrapMinusPiToPi(0.0);

  vision_common::Trajectory reference_trajectory =
      trajectory_generation_helper::polynomials::computeTimeOptimalTrajectory(
          start_state, end_state, order_, max_vel_, max_thrust_,
          max_roll_pitch_rate_, kExecLoopRate_);
  trajectory_generation_helper::heading::addConstantHeadingRate(
      start_state.heading, end_state.heading, &reference_trajectory);

  // auto p0 = reference_trajectory.points.begin();
  // for (int i = 0; p0 != reference_trajectory.points.end(); ++p0, ++i) {
  //   std::cout << i << ": " << p0->time_from_start.toSec() << ", "
  //             << p0->position.transpose() << std::endl;
  // }

  traj_pub_.publish(reference_trajectory.toRosMessage());    
}

void CirclePlanner::odometryCallback(const nav_msgs::Odometry& msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  state_est_.position = vision_common::geometryToEigen(msg.pose.pose.position);
  state_est_.velocity = vision_common::geometryToEigen(msg.twist.twist.linear);
  state_est_.orientation = vision_common::geometryToEigen(msg.pose.pose.orientation);

  if (!velocity_estimate_in_world_frame_) {
    state_est_.velocity = state_est_.orientation * state_est_.velocity;
  }

  ready_ = true;
}

void CirclePlanner::paramsCallback(const vision_msgs::TuneData48& msg) {
  std::lock_guard<std::mutex> lock(mutex_);

}


void CirclePlanner::clickedGoalCallback(
    const geometry_msgs::PoseStamped& msg) {

  std::lock_guard<std::mutex> lock(mutex_);

  Eigen::Vector3d xy = vision_common::geometryToEigen(msg.pose.position);
  position_goal_.x() = xy.x();
  position_goal_.y() = xy.y();
}

void CirclePlanner::testHorizontalCircle() {
  ROS_INFO("Go horizontal circle.");
  // from start to end
  vision_common::Trajectory circle_traj =
      trajectory_generation_helper::circles::computeHorizontalCircleTrajectory(
          hcircle_center_, hradius_, max_vel_, hphi_start_, hphi_end_,
          kExecLoopRate_);

  // double start_heading = vision_common::wrapMinusPiToPi(hphi_start_ + M_PI);
  // double end_heading = vision_common::wrapMinusPiToPi(hphi_end_ + M_PI);

  double start_heading = 0.0;
  double end_heading = 0.0;

  trajectory_generation_helper::heading::addConstantHeadingRate(
      start_heading, end_heading, &circle_traj);

  traj_pub_.publish(circle_traj.toRosMessage());     
}


bool CirclePlanner::loadParameters() {
  // TODO: load parameters
#define GET_PARAM(name) \
  if (!vision_common::getParam(#name, name, pnh_)) return false

#define GET_PARAM_(name) \
  if (!vision_common::getParam(#name, name##_, pnh_)) return false

  GET_PARAM_(order);
  GET_PARAM_(max_vel);
  GET_PARAM_(max_thrust);
  GET_PARAM_(max_roll_pitch_rate);

  GET_PARAM_(hradius);
  GET_PARAM_(hphi_start);
  GET_PARAM_(hphi_end);
  GET_PARAM_(hnum_circles);
  hphi_start_ = vision_common::deg2rad(hphi_start_);
  hphi_end_ = vision_common::deg2rad(hphi_end_);

#undef GET_PARAM
#undef GET_PARAM_

  std::vector<double> pos(3);
  if (!pnh_.getParam("hcircle_center", pos)) {
    ROS_WARN("Circle Tracking: hcircle_center is not set.");
  } else {
    hcircle_center_ = Eigen::Vector3d(pos[0], pos[1], pos[2]);

    ROS_INFO_STREAM("[" << pnh_.getNamespace() << "] "
                        << "hcircle_center"
                        << " = " << hcircle_center_.transpose());
  }

  return true;
}

}