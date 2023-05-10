#pragma once

#include <flight_system/filght_system.h>
#include <vision_msgs/ControlCommand.h>

namespace fs {

class RotorSSystem : public FlightSystem {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  RotorSSystem();
  
  ~RotorSSystem();

  void configTopics();

  void publishTopics();

  void sendBodyRates(const vision_common::ControlCommand& command);

  void sendPosition(
    const Eigen::Vector3d& position,
    const Eigen::Quaterniond& orientation);            
};

}