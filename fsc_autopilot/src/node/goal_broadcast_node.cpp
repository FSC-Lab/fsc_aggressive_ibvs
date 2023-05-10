/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4
 * Pro Flight Stack and tested in Gazebo SITL
 */

#include <geometry_msgs/PoseStamped.h>
#include <vision_common/math_common.h>
#include <vision_common/conversion_common.h>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <iostream>

geometry_msgs::PoseStamped current_goal;

int main(int argc, char **argv) {
  ros::init(argc, argv, "goal_broadcast_node");
  ros::NodeHandle nh;

  ros::Publisher goal_pub =
      nh.advertise<geometry_msgs::PoseStamped>("autopilot/reference_pose", 1);

  // the setpoint publishing rate MUST be faster than 2Hz
  ros::Rate rate(20.0);

  while (ros::ok()) {
    double g_x = 0.0, g_y = 0.0, g_z = 0.0, yaw = 0.0;
    bool pub_yes = false;

    while (!pub_yes) {
      std::cout << "please input the goal position" << std::endl;
      std::cin >> g_x >> g_y >> g_z >> yaw;
      std::cout << "goal recieved" << std::endl;
      std::cout << "x:" << g_x << " y:" << g_y  << " z:" << g_z  << " yaw:" << yaw << std::endl;
      std::cout << "please make sure the target is normal: 1 for publishment, "
                   "0 for selecting the goal target again"
                << std::endl;
      std::cin >> pub_yes;
    }

    current_goal.header.stamp = ros::Time::now();
    current_goal.header.frame_id = "world";
    current_goal.pose.position.x = g_x;
    current_goal.pose.position.y = g_y;
    current_goal.pose.position.z = g_z;
    current_goal.pose.orientation = vision_common::eigenToGeometry(
      Eigen::Quaterniond(Eigen::AngleAxisd(vision_common::deg2rad(yaw), Eigen::Vector3d::UnitZ())));

    goal_pub.publish(current_goal);
    std::cout << "goal_published" << std::endl;

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
