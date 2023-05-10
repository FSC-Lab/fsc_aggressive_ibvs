#pragma once

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/PointCloud.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CompanionProcessStatus.h>
#include <mavros_msgs/Param.h>
#include <mavros_msgs/ParamGet.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/WaypointList.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/UInt8.h>
#include <vision_common/math_common.h>
#include <vision_common/parameter_helper.h>
#include <vision_common/state_estimate.h>
#include <vision_common/trajectory_point.h>
#include <vision_common/control_command.h>
#include <vision_common/visual_feature.h>
#include <vision_common/visual_features.h>
#include <vision_msgs/StateEstimate.h>
#include <vision_msgs/TrajectoryPoint.h>
#include <vision_msgs/ControlCommand.h>
#include <vision_msgs/AutopilotFeedback.h>
#include <vision_msgs/TuneData12.h>
#include <vision_msgs/TuneData24.h>
#include <vision_msgs/TuneData48.h>
#include <vision_msgs/RcFeedback.h>

#include <autopilot_monitor/BasicTuneConfig.h>
#include <dynamic_reconfigure/server.h>

#include <math.h>
#include <Eigen/Dense>
#include <atomic>
#include <boost/bind.hpp>
#include <mutex>
#include <thread>

namespace autopilot_monitor {

class AutopilotMonitor {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  AutopilotMonitor();
  ~AutopilotMonitor();

  void InitializeNode();
  void startServices();
  void startNode();
  bool loadParameters();
  
 private:
  void pubLoopCallback(const ros::TimerEvent &event);
  void autopilotFeedbackCallback(const vision_msgs::AutopilotFeedback &msg);
  void optitrackCallback(const geometry_msgs::PoseStamped& msg);
  void thrustCommandCallback(const mavros_msgs::AttitudeTarget& msg);
  void pixhawkModeCallback(const mavros_msgs::State& msg);
  void rcCallback(const vision_msgs::RcFeedback& msg);
  void pointCallback(const sensor_msgs::PointCloud& msg);
  void dynamicReconfigureCallback(autopilot_monitor::BasicTuneConfig &config,
                                  uint32_t level);

  void parseAutopilotSettings(const vision_msgs::AutopilotFeedback& msg);
  void parseStateEstimate(const vision_msgs::StateEstimate& state_estimate);
  void parseReferenceState(const vision_msgs::TrajectoryPoint& reference_state);
  void parseStateAndReference(const vision_msgs::StateEstimate& state_estimate,
                              const vision_msgs::TrajectoryPoint& reference_state);
  void parseControlCommand(const vision_msgs::ControlCommand& control_command);
  void parsePixhawkMode(const mavros_msgs::State& msg);
  void parseRcFeedback(const vision_msgs::RcFeedback& msg);
  void parsePoints();

 private:
  /* ROS Utils */
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber pixhawk_mode_sub_;
  ros::Subscriber rc_sub_;
  ros::Subscriber point_sub_;
  ros::Subscriber feedback_sub_;
  ros::Subscriber optitrack_sub_;
  ros::Subscriber mavros_att_setpoint_sub_;
  ros::Subscriber mavros_pos_setpoint_sub_;
  vision_msgs::AutopilotFeedback feedback_msg_;
  bool autopilot_feedback_received_;
  bool optitrack_received_;
  bool mavros_att_setpoint_received_ = false;

  mavros_msgs::State pixhawk_mode_msg_;
  bool pixhawk_mode_received_ = false;

  vision_msgs::RcFeedback rc_msg_;
  double scale1_;
  double scale2_;
  bool rc_received_ = false;

  vision_common::VisualFeature point_;
  Eigen::Vector3d global_point_;
  bool point_available_ = false;
  double image_u_bound_ = 0.48;
  double image_v_bound_ = 0.25;

  double normalized_thrust_;
  int counter = 0;

  std::atomic<bool> should_exit_;

  ros::Timer pub_loop_timer_;
  ros::CallbackQueue pub_loop_queue_;
  std::unique_ptr<ros::AsyncSpinner> pub_loop_spinner_;
  std::mutex main_mutex_;

  /* Reconfigure */
  dynamic_reconfigure::Server<autopilot_monitor::BasicTuneConfig> *server_ =
      nullptr;
  autopilot_monitor::BasicTuneConfig rqt_param_config_;
  boost::recursive_mutex config_mutex_;

  double max_speed_;
  std::string last_flight_mode_name_;

  double max_distance_;
  double min_distance_;
  static constexpr double kPubLoopFrequency_ = 5.0;
};

}  // namespace autopilot_monitor