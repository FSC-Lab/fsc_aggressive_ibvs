#include <ros/ros.h>

#include "param_tuner/param_tuner.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "param_tuner_node");

  ROS_INFO("Initialize PX4 remote tune node...");
  tune::ParamTuner tune_manager;

  ros::spin();

  return 0;
}