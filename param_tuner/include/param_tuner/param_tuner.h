#pragma once

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <vision_common/math_common.h>
#include <vision_common/parameter_helper.h>
#include <vision_msgs/TuneData12.h>
#include <vision_msgs/TuneData24.h>
#include <vision_msgs/TuneData48.h>
#include <Eigen/Dense>

#include <dynamic_reconfigure/server.h>
#include <param_tuner/ParamTunerConfig.h>

#include <math.h>
#include <boost/bind.hpp>
#include <mutex>

namespace tune {

class ParamTuner {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ParamTuner();
  ~ParamTuner();

  void InitializeNode();
  void startServices();

 private:
  void dynamicReconfigureCallback(param_tuner::ParamTunerConfig& config,
                                  uint32_t level);

 private:
  /* ROS Utils */
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher params_pub_;
  ros::Publisher arm_pub_;
  ros::Publisher reset_pub_;

  /* ROS Utils */
  vision_msgs::TuneData48 tune_data_48_msg_;

  /* Reconfigure */
  dynamic_reconfigure::Server<param_tuner::ParamTunerConfig>* server_ =
      nullptr;
  param_tuner::ParamTunerConfig rqt_param_config_;
  boost::recursive_mutex config_mutex_;
};

}  // namespace tune