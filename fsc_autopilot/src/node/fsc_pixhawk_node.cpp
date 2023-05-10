#include <ros/ros.h>
#include <fsc_autopilot/fsc_autopilot.h>
#include <flight_system/pixhawk_system.h>
#include <mpc/mpc_controller.h>
#include <mpc/mpc_params.h>

template class fsc::Autopilot<fs::PixhawkSystem,
                              mpc::MpcController<float>,
                              mpc::MpcParams<float>>;

int main(int argc, char **argv) {
  ros::init(argc, argv, "fsc_pixhawk_node");
  
  fsc::Autopilot<fs::PixhawkSystem,
                 mpc::MpcController<float>,
                 mpc::MpcParams<float>> autopilot;

  ros::spin();

  return 0;
}