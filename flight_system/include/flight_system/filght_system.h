#pragma once

#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <atomic>
#include <mutex>
#include <thread>
#include <vision_common/parameter_helper.h>
#include <vision_common/control_command.h>

namespace fs {

class FlightSystem {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  FlightSystem(
      const double max_thrust = 20.0,
      const double min_thrust = 0.0,
      const double thrust_ratio = 1.0);

  virtual ~FlightSystem();

  virtual void init(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);

  virtual void stateLoopCallback(const ros::TimerEvent& event);

  virtual void setAutopilotState(const int autopilot_state);

  virtual void configTopics() = 0;

  virtual void publishTopics() = 0;

  virtual void sendBodyRates(const vision_common::ControlCommand& command) = 0;

  virtual void sendPosition(
    const Eigen::Vector3d& position,
    const Eigen::Quaterniond& orientation) = 0;

  virtual bool armed() { return armed_; }

  virtual bool ready() { return ready_; }

  virtual void setThrustRatio(const double ratio) { thrust_ratio_ = ratio; }

  virtual void setMaxThrust(const double max_thrust) { max_thrust_ = max_thrust; }

  virtual void setMinThrust(const double min_thrust) { min_thrust_ = min_thrust; }

 protected:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher autopilot_state_pub_;
  ros::Publisher bodyrates_pub_;
  ros::Publisher position_pub_;

  ros::Timer state_loop_timer_;
  ros::CallbackQueue state_loop_queue_;
  std::unique_ptr<ros::AsyncSpinner> state_loop_spinner_;

  std::atomic<bool> should_exit_;
  std::mutex state_mutex_;
  int autopilot_state_;
  double max_thrust_;
  double min_thrust_;
  double thrust_ratio_;
  bool armed_;
  bool ready_;

  static constexpr double kStateLoopFrequency_ = 5;
};

}  // namespace fs