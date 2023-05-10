#include <flight_system/rotors_system.h>

namespace fs {

RotorSSystem::RotorSSystem(): FlightSystem() {
  armed_ = true;
  ready_ = true;
}

RotorSSystem::~RotorSSystem() {}

void RotorSSystem::configTopics() {
  bodyrates_pub_ =
      nh_.advertise<vision_msgs::ControlCommand>("autopilot/control_command", 1);
}

void RotorSSystem::publishTopics() {}

// TODO: assign command
void RotorSSystem::sendBodyRates(const vision_common::ControlCommand& command) {
  if (should_exit_) {
    return;
  }

  bodyrates_pub_.publish(command.toRosMessage());
}

void RotorSSystem::sendPosition(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation) {
  if (should_exit_) {
    return;
  }
}

}  // namespace fs