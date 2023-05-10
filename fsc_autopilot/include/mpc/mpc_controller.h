#pragma once

#include <thread>
#include <iostream>
#include <math.h>

#include <Eigen/Eigen>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Path.h>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>

#include <vision_common/math_common.h>
#include <vision_common/normal_vector.h>
#include <vision_common/visual_features.h>
#include <vision_common/control_command.h>
#include <vision_common/state_estimate.h>
#include <vision_common/trajectory.h>
#include <vision_common/trajectory_point.h>

#include <mpc/mpc_wrapper.h>
#include <mpc/mpc_params.h>

namespace mpc {

template<typename T>
class MpcController {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  //TODO: MPC
  static_assert(kStateSize == 18,
                "MpcController: Wrong model size. Number of states does not match.");
  static_assert(kInputSize == 6,
                "MpcController: Wrong model size. Number of inputs does not match.");

  MpcController(const ros::NodeHandle& nh,
                     const ros::NodeHandle& pnh,
                     const std::string& topic = "autopilot/trajectory_predicted");

  MpcController() : MpcController(ros::NodeHandle(), ros::NodeHandle("~")) {}

  ~MpcController();
  
  vision_common::ControlCommand off();

  bool run(const vision_common::StateEstimate& state_est,
           const vision_common::Trajectory& reference_trajectory,
           MpcParams<T>& params,
           std::list<vision_common::ControlCommand>& command_queue,
           std::vector<vision_common::StateEstimate>& predicted_states);

  bool run(const vision_common::StateEstimate& state_est,
           const vision_common::Trajectory& reference_trajectory,
           MpcParams<T>& params,
           std::list<vision_common::ControlCommand>& command_queue);

  void reset();
  
 private:
  
//   void offCallback(const std_msgs::Empty::ConstPtr& msg);

  bool setState(
      const vision_common::StateEstimate& state_est);

  bool setReference(const vision_common::Trajectory& reference_trajectory);

  bool getControlCommand(
      const Eigen::Ref<const Eigen::Matrix<T, kStateSize, 1>> state,
      const Eigen::Ref<const Eigen::Matrix<T, kInputSize, 1>> input,
      ros::Time& time, vision_common::ControlCommand& command);

  bool getPredictedState(
      const Eigen::Ref<const Eigen::Matrix<T, kStateSize, 1>> state,
      ros::Time& time, vision_common::StateEstimate& predicted_state);

  void preparationThread();

  bool setNewParams(MpcParams<T>& params);

  // Handles
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  // Subscribers and publisher.
//   ros::Subscriber sub_autopilot_off_;
//   ros::Publisher pub_predicted_trajectory_;

  // Parameters
  MpcParams<T> params_;

  // MPC
  MpcWrapper<T> mpc_wrapper_;

  // Preparation Thread
  std::thread preparation_thread_;

  // Variables
  T timing_feedback_, timing_preparation_;
  bool solve_from_scratch_;
  Eigen::Matrix<T, kStateSize, 1> est_state_;
  Eigen::Matrix<T, kCostSize, kSamples + 1> reference_states_;
  Eigen::Matrix<T, kInputSize, kSamples + 1> reference_inputs_;
  Eigen::Matrix<T, kStateSize, kSamples + 1> predicted_states_;
  Eigen::Matrix<T, kInputSize, kSamples> predicted_inputs_;
};


}
