
#include <ros/ros.h>
#include <fsc_autopilot/fsc_autopilot.h>
#include <flight_system/flightgoggles_system.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "fsc_flightgoggles_node");
  
  fsc::Autopilot<fs::FlightGogglesSystem> autopilot;

  ros::spin();

  return 0;
}