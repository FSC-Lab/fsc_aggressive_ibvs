#include <flight_system/filght_system.h>

namespace fs {

FlightSystem::FlightSystem(
      const double max_thrust,
      const double min_thrust,
      const double thrust_ratio)
    : should_exit_(false),
      autopilot_state_(0), 
      max_thrust_(max_thrust),
      min_thrust_(min_thrust),
      thrust_ratio_(thrust_ratio),
      armed_(false),
      ready_(false) {}

FlightSystem::~FlightSystem() { should_exit_ = true; }

void FlightSystem::init(const ros::NodeHandle& nh, const ros::NodeHandle& pnh) {
  should_exit_ = false;
  nh_ = nh;
  pnh_ = pnh;

#define GET_PARAM(name) \
  if (!vision_common::getParam(#name, name, pnh_)) return
#define GET_PARAM_(name) \
  if (!vision_common::getParam(#name, name##_, pnh_)) return

  GET_PARAM_(max_thrust);
  GET_PARAM_(min_thrust);
  GET_PARAM_(thrust_ratio);

#undef GET_PARAM
#undef GET_PARAM_

  configTopics();

  ros::TimerOptions state_loop_options(
      ros::Duration(1.0 / kStateLoopFrequency_),
      boost::bind(&FlightSystem::stateLoopCallback, this, _1),
      &state_loop_queue_);
  state_loop_timer_ = nh_.createTimer(state_loop_options);
  state_loop_spinner_.reset(new ros::AsyncSpinner(1, &state_loop_queue_));
  state_loop_spinner_->start();
}

void FlightSystem::stateLoopCallback(const ros::TimerEvent& event) {
  if (should_exit_) {
    return;
  }

  std::lock_guard<std::mutex> lock(state_mutex_);
  publishTopics();
}

void FlightSystem::setAutopilotState(const int autopilot_state) {
  if (should_exit_) {
    return;
  }

  std::lock_guard<std::mutex> lock(state_mutex_);
  autopilot_state_ = autopilot_state;
};

}  // namespace fs
