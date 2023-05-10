#include <ros/ros.h>
#include <vision_msgs/FlightMode.h>
#include <iostream>
#include <string>

int main(int argc, char **argv) {
  ros::init(argc, argv, "flight_mode_publisher_node");
  
  ros::NodeHandle nh;
  vision_msgs::FlightMode fm_msg;

  ros::Publisher fm_pub =
      nh.advertise<vision_msgs::FlightMode>("autopilot/flight_mode", 1);

  ros::Rate rate(20.0);
  while (ros::ok()) {
    int flight_mode  = 0.0;
    bool pub_yes = false;

    while (!pub_yes) {
      std::cout << "Please specify a flight mode" << std::endl;
      std::cout << "----------------------------" << std::endl;
      std::cout << "0: POSITION_TRACK" << std::endl;
      std::cout << "1: TRAJECTORY_TRACK" << std::endl;
      std::cout << "2: VELOCITY_TRACK" << std::endl;
      std::cout << "3: FEATURE_TRACK" << std::endl;
      std::cout << "4: FEATURE_REACH" << std::endl;
      std::cout << "5: HOME" << std::endl;
      std::cout << "6: LAND" << std::endl;
      std::cout << "----------------------------" << std::endl;

      std::cin >> flight_mode;
      // std::cout << "new flight mode recieved" << std::endl;

      std::string flight_mode_name;
      switch (flight_mode) {
        case 0:
          fm_msg.flight_mode = fm_msg.POSITION_TRACK;
          flight_mode_name = "POSITION_TRACK";
          break;
        case 1:
          fm_msg.flight_mode = fm_msg.TRAJECTORY_TRACK;
          flight_mode_name = "TRAJECTORY_TRACK";
          break;
        case 2:
          fm_msg.flight_mode = fm_msg.VELOCITY_TRACK;
          flight_mode_name = "VELOCITY_TRACK";
          break;
        case 3:
          fm_msg.flight_mode = fm_msg.FEATURE_TRACK;
          flight_mode_name = "FEATURE_TRACK";
          break;
        case 4:
          fm_msg.flight_mode = fm_msg.FEATURE_REACH;
          flight_mode_name = "FEATURE_REACH";
          break;
        case 5:
          fm_msg.flight_mode = fm_msg.HOME;
          flight_mode_name = "HOME";
          break;
        case 6:
          fm_msg.flight_mode = fm_msg.LAND;
          flight_mode_name = "LAND";
          break;
      }
 
      std::cout << flight_mode_name << " is selected.\n" << std::endl;   

      std::cout << "ready to enther this flight mode? 1 for publishment, "
                   "0 for selecting the flight mode again"
                << std::endl;
      std::cin >> pub_yes;
    }

    if (pub_yes) {
      fm_msg.header.stamp = ros::Time::now();
      fm_pub.publish(fm_msg);
      std::cout << "publish message." << std::endl;
    } else {
      std::cout << "enter the flight mode again." << std::endl;
    }

    ros::spinOnce();
    rate.sleep();
  }
  ros::spin();

  return 0;
}