#include <flight_system/pixhawk_system.h>

namespace fs {

PixhawkSystem::PixhawkSystem() : PixhawkSystem(1.0, 0.0, 1.0) {}

PixhawkSystem::PixhawkSystem(
    const double max_thrust,
    const double min_thrust,
    const double thrust_ratio)
    : FlightSystem(max_thrust, min_thrust, thrust_ratio),
      pixhawk_mode_(PixhawkMode::NONE),
      pixhawk_params_(),
      offboard_triggered_(false),
      motion_capture_ready_(false),
      motion_capture_timeout_(0.1),
      use_motion_capture_(false) {}

PixhawkSystem::~PixhawkSystem() {}

void PixhawkSystem::configTopics() {

  if (!loadParameters()) {
    ROS_ERROR("[%s] Could not load PIXHAWK_SYSTEM parameters.", pnh_.getNamespace().c_str());
    ros::shutdown();
    return;
  }

  pixhawk_mode_sub_ = nh_.subscribe("/mavros/state", 1,
                                    &PixhawkSystem::pixhawkModeCallback, this);

  pixhawk_param_sub_ =
      nh_.subscribe("/mavros/param/param_value", 1,
                    &PixhawkSystem::pixhawkParamsCallback, this);

  if (use_motion_capture_) {
    optitrack_sub_ = nh_.subscribe<const geometry_msgs::PoseStamped&>(
        "optitrack_topic", 1, &PixhawkSystem::optitrackCallback, this);
  }

  autopilot_state_pub_ = nh_.advertise<mavros_msgs::CompanionProcessStatus>(
      "/mavros/companion_process/status", 1);

  bodyrates_pub_ = nh_.advertise<mavros_msgs::AttitudeTarget>(
      "/mavros/setpoint_raw/attitude", 1);

  position_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
      "/mavros/setpoint_position/local", 1);
}

void PixhawkSystem::publishTopics() {

  if (use_motion_capture_) {
    if ((ros::Time::now() - time_last_motion_capture_received_) > ros::Duration(motion_capture_timeout_)) {
      motion_capture_ready_ = false;
    }
  }

  mavros_msgs::CompanionProcessStatus status_msg;
  status_msg.header.stamp = ros::Time::now();
  status_msg.component = 196;  // MAV_COMPONENT_ID_AVOIDANCE
  status_msg.state = autopilot_state_;
  autopilot_state_pub_.publish(status_msg);

  // PX4 specialized offboard triggers if AutopilotState::STANDBY
  // if (autopilot_state_ == 3 && !ready()) {
  //   //TODO:CHANGE
  //   // sendBodyRates(Eigen::Vector3d::Zero(), 0.02);
  // }
}

void PixhawkSystem::sendBodyRates(const vision_common::ControlCommand& command) {
  if (should_exit_) {
    return;
  }

  if (autopilot_state_ == 4 && !ready()) {
    return;
  }

  double normalized_thrust = normalizeThrust(command.collective_thrust, max_thrust_, min_thrust_);
  
  mavros_msgs::AttitudeTarget att_msg;
  att_msg.header.frame_id = "base_footprint";
  att_msg.header.stamp = ros::Time::now();
  att_msg.orientation.w = 1.0;
  att_msg.orientation.x = 0.0;
  att_msg.orientation.y = 0.0;
  att_msg.orientation.z = 0.0;
  att_msg.body_rate.x = command.bodyrates.x();
  att_msg.body_rate.y = command.bodyrates.y();
  att_msg.body_rate.z = command.bodyrates.z();
  att_msg.thrust = normalized_thrust;
  // att_msg.type_mask = 128;
  att_msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;
  att_msg.header.stamp = ros::Time::now();

  // std::cout << "Normalized Thrust: " << normalized_thrust << std::endl;

  bodyrates_pub_.publish(att_msg);
}

void PixhawkSystem::sendPosition(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation) {
  geometry_msgs::PoseStamped msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "/local_origin";
  msg.pose.position = vision_common::vectorToPoint(vision_common::eigenToGeometry(position));
  msg.pose.orientation = vision_common::eigenToGeometry(orientation);
  position_pub_.publish(msg); 
}

bool PixhawkSystem::ready() {
  if (use_motion_capture_) {
    ready_ = offboard_triggered_ && motion_capture_ready_;
    return ready_;
  } else {
    ready_ = offboard_triggered_;
    return ready_;
  }

  return true;
}

void PixhawkSystem::pixhawkModeCallback(const mavros_msgs::State& msg) {
  if (should_exit_) {
    return;
  }

  std::lock_guard<std::mutex> lock(state_mutex_);
  // set armed flag
  armed_ = msg.armed;
  // set ready flag
  if (msg.mode == "OFFBOARD") {
    offboard_triggered_ = true;
  } else {
    offboard_triggered_ = false;
  }

  if (msg.mode == "MANUAL") {
    pixhawk_mode_ = PixhawkMode::MANUAL;
  } else if (msg.mode == "ACRO") {
    pixhawk_mode_ = PixhawkMode::ACRO;
  } else if (msg.mode == "ALTCTL") {
    pixhawk_mode_ = PixhawkMode::ALTCTL;
  } else if (msg.mode == "POSCTL") {
    pixhawk_mode_ = PixhawkMode::POSCTL;
  } else if (msg.mode == "OFFBOARD") {
    pixhawk_mode_ = PixhawkMode::OFFBOARD;
  } else if (msg.mode == "STABILIZED") {
    pixhawk_mode_ = PixhawkMode::STABILIZED;
  } else if (msg.mode == "RATTITUDE") {
    pixhawk_mode_ = PixhawkMode::RATTITUDE;
  } else if (msg.mode == "AUTO.MISSION") {
    pixhawk_mode_ = PixhawkMode::AUTO_MISSION;
  } else if (msg.mode == "AUTO.LOITER") {
    pixhawk_mode_ = PixhawkMode::AUTO_LOITER;
  } else if (msg.mode == "AUTO.RTL") {
    pixhawk_mode_ = PixhawkMode::AUTO_RTL;
  } else if (msg.mode == "AUTO.LAND") {
    pixhawk_mode_ = PixhawkMode::AUTO_LAND;
  } else if (msg.mode == "AUTO.RTGS") {
    pixhawk_mode_ = PixhawkMode::AUTO_RTGS;
  } else if (msg.mode == "AUTO.READY") {
    pixhawk_mode_ = PixhawkMode::AUTO_READY;
  } else if (msg.mode == "AUTO.TAKEOFF") {
    pixhawk_mode_ = PixhawkMode::AUTO_TAKEOFF;
  } else {
    pixhawk_mode_ = PixhawkMode::NONE;
  }
}

void PixhawkSystem::pixhawkParamsCallback(const mavros_msgs::Param& msg) {
  if (should_exit_) {
    return;
  }

}

void PixhawkSystem::optitrackCallback(
    const geometry_msgs::PoseStamped& msg) {
  if (should_exit_) {
    return;
  }

  std::lock_guard<std::mutex> lock(state_mutex_);

  motion_capture_ready_ = true;
  time_last_motion_capture_received_ = ros::Time::now();
}

double PixhawkSystem::normalizeThrust(const double thrust,
                                      const double max_thrust,
                                      const double min_thrust) {
  return std::min(std::max(thrust_ratio_ * thrust, min_thrust), max_thrust) / max_thrust;
}

bool PixhawkSystem::loadParameters() {
  ROS_INFO("-----------Loading PIXHAWK_SYSTEM Parameters-----------");

#define GET_PARAM(name) \
  if (!vision_common::getParam(#name, name, pnh_)) return false
#define GET_PARAM_(name) \
  if (!vision_common::getParam(#name, name##_, pnh_)) return false

  // GET_PARAM_(use_motion_capture);
  // if (use_motion_capture_) {
  //   GET_PARAM_(motion_capture_timeout);
  // }

#undef GET_PARAM
#undef GET_PARAM_
  return true;
}

}  // namespace fs