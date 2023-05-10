#include <flight_system/flightgoggles_system.h>

namespace fs {

FlightGogglesSystem::FlightGogglesSystem(): FlightSystem() {}

FlightGogglesSystem::~FlightGogglesSystem() {}

void FlightGogglesSystem::configTopics() {
  armed_sub_ = nh_.subscribe("/uav/input/arm", 1,
                             &FlightGogglesSystem::armedCallback, this);

  bodyrates_pub_ =
      nh_.advertise<mav_msgs::RateThrust>("/uav/input/rateThrust", 10);
}

void FlightGogglesSystem::publishTopics() {}

void FlightGogglesSystem::sendBodyRates(const Eigen::Vector3d& bodyrate,
                                        const double thrust) {
  if (should_exit_) {
    return;
  }
  
  mav_msgs::RateThrust msg;
  msg.header.frame_id = "uav/imu";
  msg.header.stamp = ros::Time::now();
  msg.angular_rates.x = bodyrate.x();
  msg.angular_rates.y = bodyrate.y();
  msg.angular_rates.z = bodyrate.z();
  msg.thrust.x = 0;
  msg.thrust.y = 0;
  msg.thrust.z = thrust;
  bodyrates_pub_.publish(msg);
}

void FlightGogglesSystem::armedCallback(const std_msgs::Empty& msg) {
  if (should_exit_) {
    return;
  }

  std::lock_guard<std::mutex> lock(state_mutex_);
  armed_ = true;
  ready_ = true;
}

}  // namespace fs