#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <vision_common/visual_feature.h>
#include <vision_common/state_estimate.h>
#include <vision_common/normal_vector.h>

#include <Eigen/Dense>
#include <iostream>
#include <vector>

#include "vision_msgs/VisualFeature.h"
#include "vision_msgs/StateEstimate.h"

using namespace vision_common;
using namespace std;

#define rad2deg 180.0 / M_PI
#define deg2rad M_PI / 180.0

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_vision_common");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::cout << "--------Test Normal Vector--------" << std::endl;

  /***# Test Normal Vector #***/
  Eigen::Quaterniond q0(1, 0, 0, 0);
  NormalVector normal_vector0(q0);
  // normal_vector0.print();
  std::cout << normal_vector0.getVec().transpose() << std::endl;

  Eigen::Vector3d rpy(30 * deg2rad, 60 * deg2rad, 90 * deg2rad);
  Eigen::Quaterniond dq = vision_common::eulerAnglesZYXToQuaternion(rpy);
  // Rotate normal_vector0 by dq: dq * orientation
  normal_vector0.rotated(dq);
  // normal_vector0.print();

  NormalVector normal_vector1(normal_vector0);
  // normal_vector1.print();

  std::cout << "--------BoxPlus--------" << std::endl;
  Eigen::Vector2d u1(10 * deg2rad, 20 * deg2rad);
  NormalVector normal_vector2;
  // Rotate normal_vector2 by u1. normal_vector1 + u1 = normal_vector2
  normal_vector1.boxPlus(u1, normal_vector2);
  // normal_vector2.print();
  std::cout << "rpy: "
            << (vision_common::quaternionToEulerAnglesZYX(normal_vector2.orientation) *
                rad2deg)
                   .transpose()
            << std::endl;

  std::cout << "--------BoxMinus--------" << std::endl;
  Eigen::Vector2d u2, u3;
  // Compute normal_vector2 - normal_vector1 = u2. u2 is represented in the normal_vector1 frame
  normal_vector2.boxMinus(normal_vector1, u2);
  std::cout << "u2: " << u2.transpose() * rad2deg << std::endl;

  std::cout << "--------Comparison--------" << std::endl;
  Eigen::Quaterniond q1 = q0 * dq;
  std::cout << (q1 * Eigen::Vector3d(0, 0, 1)).transpose() << std::endl;

  // Euler angle can approximate the angle axis in small-angle cases
  Eigen::Vector3d rpy_u(10 * deg2rad, 20 * deg2rad, 0 * deg2rad);
  Eigen::Quaterniond dq_rpy =
      vision_common::eulerAnglesZYXToQuaternion(rpy_u);
  Eigen::Vector3d aa(10 * deg2rad, 20 * deg2rad, 0 * deg2rad);
  Eigen::Quaterniond dq_aa = angleAxisToQuaternion(aa);

  Eigen::Quaterniond q2 = q1 * dq_aa;
  std::cout << (q2 * Eigen::Vector3d(0, 0, 1)).transpose() << std::endl;
  std::cout << "rpy: "
            << (vision_common::quaternionToEulerAnglesZYX(q2) * rad2deg)
                   .transpose()
            << std::endl;
  std::cout << "--------Test Visual Feature--------" << std::endl;
  /***# Test Visual Feature #***/
  VisualFeature f1;
  // f1.normal_vector.print();
  std::cout << f1.distance << std::endl;

  double distance1 = 0.2;
  VisualFeature f2(normal_vector1, distance1);
  // f2.normal_vector.print();
  std::cout << f2.distance << std::endl;

  VisualFeature f3(normal_vector1.getVec(), distance1);
  // f3.normal_vector.print();
  std::cout << f3.distance << std::endl;

  VisualFeature f4(normal_vector1.orientation, distance1);
  // f4.normal_vector.print();
  std::cout << f4.distance << std::endl;

  // /***# Test State Estimate #***/
  // vision_common::StateEstimate odom1;
  // // odom1.coordinate_frame = vision_common::StateEstimate::CoordinateFrame::WORLD;

  // std::cout << "odom1 timestamp: " << odom1.timestamp << std::endl;
  // std::cout << "odom1 coordinate_frame: " << int(odom1.coordinate_frame) << std::endl;
  // std::cout << "odom1 position: " << odom1.position.transpose() << std::endl;
  // std::cout << "odom1 velocity: " << odom1.velocity.transpose() << std::endl;
  // std::cout << "odom1 orientation: " << odom1.orientation.vec().transpose() << std::endl;

  // std::vector<VisualFeature> fts;
  // fts.push_back(f1);
  // fts.push_back(f2);
  // // StateEstimate img1(odom1, fts);

  // if (!img1.isValid()) {
  //   std::cout << "img1 is invalid." << std::endl;
  // }

  // std::cout << "--------Test Visual Feature--------" << std::endl;
  // /***# Test Message Publishment #***/
  // ros::Publisher ftr_pub;
  // ftr_pub = nh.advertise<vision_msgs::VisualFeature>("visual_feature", 1);

  // ros::Publisher img_pub;
  // img_pub = nh.advertise<vision_msgs::StateEstimate>("state_estimate", 1);


  // ros::Rate loop_rate(1.0);
  // while (ros::ok()) {
  //   loop_rate.sleep();
  //   std::cout << "f2 w: " << f2.normal_vector.orientation.w() << std::endl;
  //   std::cout << "f2 xyz: " << f2.normal_vector.orientation.vec().transpose() << std::endl;
  //   ftr_pub.publish(f2.toRosMessage());
  //   img_pub.publish(img1.toRosMessage());
  // }

  ros::spin();

  return 0;
}
