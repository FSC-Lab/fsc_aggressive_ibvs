#include <ros/ros.h>

#include "autopilot_monitor/autopilot_monitor.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "autopilot_monitor_node");

  ROS_INFO("Initialize PX4 remote tune node...");
  autopilot_monitor::AutopilotMonitor autopilot_monitor;

  ros::spin();

  return 0;
}