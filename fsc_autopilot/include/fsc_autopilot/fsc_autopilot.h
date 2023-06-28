#pragma once

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int8.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <Eigen/Dense>

#include <vision_common/control_command.h>
#include <vision_common/conversion_common.h>
#include <vision_common/math_common.h>
#include <vision_common/normal_vector.h>
#include <vision_common/parameter_helper.h>
#include <vision_common/state_estimate.h>
#include <vision_common/trajectory.h>
#include <vision_common/trajectory_point.h>
#include <vision_common/visual_feature.h>
#include <vision_common/visual_features.h>
#include <vision_common/gate_feature.h>
#include <vision_common/gate_feature_array.h>
#include <vision_common/distparam_trajectory.h>
#include <state_predictor/state_predictor.h>
#include <trajectory_generation_helper/heading_trajectory_helper.h>
#include <trajectory_generation_helper/polynomial_trajectory_helper.h>

#include <vision_msgs/ControlCommand.h>
#include <vision_msgs/LowLevelFeedback.h>
#include <vision_msgs/StateEstimate.h>
#include <vision_msgs/Trajectory.h>
#include <vision_msgs/TrajectoryPoint.h>
#include <vision_msgs/TuneData48.h>
#include <vision_msgs/GateFeature.h>
#include <vision_msgs/GateFeatureArray.h>
#include <vision_msgs/VisualFeature.h>
#include <vision_msgs/VisualFeatures.h>
#include <vision_msgs/RcFeedback.h>
#include <vision_msgs/AutopilotFeedback.h>
#include <vision_msgs/FlightMode.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <atomic>
#include <boost/bind.hpp>
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>

#include <fsc_autopilot/fsc_common.h>
// #include <fsc_autopilot/distparam_trajectory.h>

#include <vision_msgs/TrajectoryRacing.h>


namespace fsc {

template <typename Tsystem, typename Tcontroller, typename Tparams>
class Autopilot {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Autopilot(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);
  Autopilot(): Autopilot(ros::NodeHandle(), ros::NodeHandle("~")) {}
  ~Autopilot();

  // System functions
  void init();
  void reset();
  void startLoop();
  void startThreads();
  bool armed();
  bool ready();

  // State functions
  void setAutopilotState(const AutopilotState& state);
  void setControlMode(const ControlMode& mode);
  void setFlightMode(const FlightMode& mode);
  AutopilotState getAutopilotState();
  ControlMode getControlMode();
  FlightMode getFlightMode();

 private:
  // Callback functions
  void commandLoopCallback(const ros::TimerEvent& event);
  void odometryCallback(const nav_msgs::Odometry& msg);
  void pointCallback(const sensor_msgs::PointCloud& msg);
  void paramsCallback(const vision_msgs::TuneData48& msg);
  void rcCallback(const vision_msgs::RcFeedback& msg);
  void referencePoseCallback(const geometry_msgs::PoseStamped& msg);
  void referenceTrajectoryCallback(const vision_msgs::Trajectory& msg);

  void flightModeCallback(const vision_msgs::FlightMode& msg);

  // Thread functions
  void sensorSynchronizationThread();
  void watchdogThread();

  // Command functions
  void sendCommand();
  void sendFirstCommand();
  void sendEmergencyCommand();
  void sendBodyRates(const Eigen::Vector3d& bodyrate, const double& thrust);
  void sendBodyRates(const vision_common::ControlCommand& command);

  void sendPosition(const Eigen::Vector3d& position,
                    const Eigen::Quaterniond& orientation);
  void solveCommands();
  bool validateCommand(const vision_common::ControlCommand& command);
  bool readyToSolve();
  bool checkStateEstimate(const vision_common::StateEstimate& state_est);
  vision_common::StateEstimate getPredictedStateEstimate( const ros::Time& time);
  void trackReferenceState(const vision_common::TrajectoryPoint& reference_state);
  void trackPosition();
  void trackTrajectory();
  void trackVelocity();
  void trackFeature();
  void home();
  void land();
  
  // MPC functions
  bool getStateEstimate();
  bool getStateForPositionTrack();
  bool getStateForFeatureTrack();
  bool checkFeatureTrack();
  bool checkFeatureReach();
  void goMPC(const vision_common::StateEstimate& state_est);

  Eigen::Vector3d computeReferencePosition(const double reference_distance);

  bool generateState(vision_common::StateEstimate& state_est,
                     Tparams& controller_params);
  bool generateStateWithVirtualFeature(vision_common::StateEstimate& state_est,
                     Tparams& controller_params);

  bool generateReferenceTrajectory(
      const vision_common::TrajectoryPoint& reference_state,
      vision_common::Trajectory& reference_trajectory,
      Tparams& controller_params);
  bool generateReferenceTrajectory(
      const ros::Duration dt,
      vision_common::Trajectory& reference_trajectory,
      Tparams& controller_params);
  visualization_msgs::Marker makeMarker(
    const Eigen::Vector3d& point,
    const Eigen::Vector3d& color);
  visualization_msgs::MarkerArray makeMarkerArray(
      const vision_common::Trajectory& trajectory,
      const Eigen::Vector3d& color);

  void publishAutopilotFeedback(
      const vision_common::StateEstimate& state_estimate,
      const vision_common::TrajectoryPoint& reference_state,
      vision_common::ControlCommand control_command,
      const ros::Duration& control_command_delay,
      const ros::Duration& control_computation_time,
      const bool success = false);

  // Parameter functions
  bool loadParameters();

  bool selectGate(
    const vision_common::GateFeatureArray& gates,
    const Eigen::Vector3d& position,
    vision_common::GateFeature& gate_selected);
  double computeGaussianScale(const double val, const double mean, const double std);
  double computeDistanceToPlane(const Eigen::Vector3d& pt, const Eigen::Vector3d& ppos, const Eigen::Quaterniond& porient);
  
 private:
  /* ROS Utils */
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Timer command_loop_timer_;
  ros::CallbackQueue command_loop_queue_;
  ros::Subscriber odom_sub_;
  ros::Subscriber point_sub_;
  ros::Subscriber params_sub_;
  ros::Subscriber rc_sub_;

  ros::Subscriber flight_mode_sub_;

  ros::Subscriber reference_pose_sub_;
  ros::Subscriber reference_trajectory_sub_;
  ros::Publisher autopilot_feedback_pub_;
  ros::Publisher vis_drone_pub_;
  ros::Publisher vis_goal_pub_;
  std::unique_ptr<ros::AsyncSpinner> command_loop_spinner_;

  /* Racing Test */
  ros::Subscriber trajracing_sub_;
  vision_common::Trajectory trajectory_racing_;
  Eigen::Vector3d local_goal_;
  vision_common::GateFeatureArray gates_;

  /* Threads */
  std::atomic<bool> should_exit_;
  std::thread sensorsyn_thread_;
  std::thread watchdog_thread_;
  std::mutex command_mutex_;
  std::mutex sensorsyn_mutex_;
  std::mutex goal_mutex_;
  std::condition_variable sensorsyn_cv_;

  /* Flight Control */
  Tsystem system_;
  Tcontroller base_controller_;
  Tparams base_controller_params_;
  Tparams feature_track_params_;
  Tparams mpcc_params_;

  double computation_time_;

  AutopilotState autopilot_state_;
  ControlMode control_mode_;
  FlightMode flight_mode_;
  bool computation_success_;
  bool mpc_forbiddened_ = false;;

  /* State Estimate */
  state_predictor::StatePredictor state_predictor_;
  vision_common::StateEstimate received_state_est_;
  vision_common::StateEstimate predicted_state_;
  vision_common::StateEstimate state_est_;
  Eigen::Vector3d euler_est_;
  double last_heading_est_;
  std::vector<vision_common::StateEstimate> mpc_states_;

  /* Point Detection */
  vision_common::VisualFeature point_;
  bool point_available_ = false;
  ros::Time time_last_point_received_;
  double point_estimate_timeout_;
  Eigen::Vector3d global_point_;
  Eigen::Vector3d last_global_point_;

  /* Point Detection */
  int order_ = 5;
  double max_planning_vel_ = 8.0;
  double max_thrust_ = 20.0;
  double max_roll_pitch_rate_ = 3.0;
  double sample_rate_ = 20.0;
  bool predicted_state_available_ = false;
  std::vector<vision_common::StateEstimate> predicted_states_;
  bool has_distparam_trajectory_ = false;
  ros::Publisher vis_trajectory_pub_;
  ros::Publisher vis_dots_pub_;
  bool warmup_mpcc_ = false;
  double feature_track_max_distance_;
  double feature_track_distance_threshold_;
  double global_point_position_change_threshold_;


  /* Flags and Times */
  bool odometry_available_;
  bool synchron_data_available_;
  bool command_available_;
  bool first_time_in_new_flight_mode_;
  ros::Time time_last_odometry_received_;

  ros::Time time_last_synchron_data_received_;
  ros::Time time_last_command_queue_updated_;
  ros::Time time_start_emergency_landing_;
  ros::Time time_start_flight_termination_;
  ros::Time time_start_trajectory_track_;
  ros::Time time_start_land_;
  ros::Time time_start_home_;

  ros::Time time_last_autopilot_feedback_published_;

  /* Commands */
  std::list<vision_common::ControlCommand> command_queue_;
  vision_common::ControlCommand command_;
  ros::Time time_command_available_;
  double max_thrust_ratio_;
  double min_thrust_ratio_;

  /* Goals */
  vision_common::TrajectoryPoint reference_state_;
  vision_common::Trajectory reference_trajectory_;
  std::list<vision_common::Trajectory> trajectory_queue_;
  
  /* Feature Track */
  double max_distance_;
  double min_distance_;
  double reference_distance_;
  double reaching_distance_;
  int invisibility_counter_;
  int point_counter_;
  bool flip_ = false;
  bool use_flip_mode_ = false;

  double cost_feature_track_;
  double cost_visibility_u_;
  double cost_visibility_v_;
  double cost_constratint_vis_;
  double cost_constratint_ttc_;

  double max_mpcc_speed_;
  double reference_speed_;
  double cost_speed_;
  double max_speed_ = 0.01;
  double height_gap_ = 0.0;

  /* Parameters */
  Eigen::Vector3d t_B_C_;
  Eigen::Quaterniond q_B_C_;
  double image_u_bound_;
  double image_v_bound_;
  vision_common::TrajectoryPoint home_state_;
  bool velocity_estimate_in_world_frame_;
  double emergency_land_thrust_;
  double max_emergency_pos_z_;
  double min_emergency_pos_z_;
  double max_emergency_pos_xy_;
  double min_emergency_pos_xy_;
  double reference_thrust_;
  double max_reference_pos_z_;
  double min_reference_pos_z_;
  double max_reference_pos_xy_;
  double min_reference_pos_xy_;
  double command_publish_frequency_;
  double watchdog_frequency_;
  double odometry_estimate_timeout_;
  double synchron_data_timeout_;
  double command_update_timeout_;
  double emergency_landing_timeout_;
  double flight_termination_timeout_;
  double land_timeout_;
  double home_timeout_;
  double predictive_control_lookahead_;
  double control_command_delay_;
  double quad_radius_;
  bool enable_trajectory_replan_;
  bool enable_moving_target_track_;
  bool enable_state_prediction_;
  bool enable_rc_tune_;
  bool print_info_;

  int current_gate_index_;
};

}  // namespace fsc

#include "./fsc_autopilot_inl.h"
