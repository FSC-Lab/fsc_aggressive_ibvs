#pragma once

#include <flight_system/filght_system.h>
#include <std_msgs/Empty.h>
#include <mav_msgs/RateThrust.h>

namespace fs {

class FlightGogglesSystem : public FlightSystem {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  FlightGogglesSystem();
  
  ~FlightGogglesSystem();

  void configTopics();

  void publishTopics();

  void sendBodyRates(const Eigen::Vector3d& bodyrate,
                             const double thrust);

 private:
  void armedCallback(const std_msgs::Empty& msg);

 private:
  ros::Subscriber armed_sub_;                   
};

}