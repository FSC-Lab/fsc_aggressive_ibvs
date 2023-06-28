
#include "circle_planner/circle_planner.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "circle_planner_node");

  ROS_INFO("Start circle_planner_node.");

  fsc::CirclePlanner planner;
  planner.run();

  ros::spin();

  return 0;
}