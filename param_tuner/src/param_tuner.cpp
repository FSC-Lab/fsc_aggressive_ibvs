#include "param_tuner/param_tuner.h"

namespace tune {

ParamTuner::ParamTuner() {
  InitializeNode();
  startServices();
}

ParamTuner::~ParamTuner() {}

void ParamTuner::InitializeNode() {
  nh_ = ros::NodeHandle("");
  pnh_ = ros::NodeHandle("~");

  params_pub_ =
      nh_.advertise<vision_msgs::TuneData48>("autopilot/tune_data_48", 1);

  arm_pub_ = nh_.advertise<std_msgs::Empty>("/uav/input/arm", 1);

  reset_pub_ = nh_.advertise<std_msgs::Empty>("/uav/input/reset", 1);
}

void ParamTuner::startServices() {
  // set up Dynamic Reconfigure Server
  server_ =
      new dynamic_reconfigure::Server<param_tuner::ParamTunerConfig>(
          config_mutex_, ros::NodeHandle("~"));
  dynamic_reconfigure::Server<
      param_tuner::ParamTunerConfig>::CallbackType f;
  f = boost::bind(&ParamTuner::dynamicReconfigureCallback, this, _1,
                  _2);
  server_->setCallback(f);
}

void ParamTuner::dynamicReconfigureCallback(
    param_tuner::ParamTunerConfig& config, uint32_t level) {
  ROS_INFO("Receive New Config!"); 
  rqt_param_config_ = config;


  tune_data_48_msg_.bdata11 = config.feature_track;
  tune_data_48_msg_.fdata6 = config.reference_distance;

  params_pub_.publish(tune_data_48_msg_);

  ROS_INFO("Send Tune Parameters!");
}

}  // namespace tune
