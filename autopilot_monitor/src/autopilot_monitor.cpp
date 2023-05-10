#include "autopilot_monitor/autopilot_monitor.h"

namespace autopilot_monitor {

AutopilotMonitor::AutopilotMonitor()
    : should_exit_(false),
      autopilot_feedback_received_(false) {
  InitializeNode();
  // startServices();
  startNode();
}

AutopilotMonitor::~AutopilotMonitor() { should_exit_ = true; }

void AutopilotMonitor::InitializeNode() {
  nh_ = ros::NodeHandle("");
  pnh_ = ros::NodeHandle("~");

  if (!loadParameters()) {
    ROS_ERROR("[%s] Could not load AutopilotMonitor parameters.", pnh_.getNamespace().c_str());
    ros::shutdown();
    return;
  }

  feedback_sub_ = nh_.subscribe("autopilot/feedback", 1,
                                 &AutopilotMonitor::autopilotFeedbackCallback, this);
  
  // optitrack_sub_ = nh_.subscribe<const geometry_msgs::PoseStamped&>(
  //     "optitrack_topic", 1, &AutopilotMonitor::optitrackCallback, this);
      
  mavros_att_setpoint_sub_ =
      nh_.subscribe("/mavros/setpoint_raw/attitude", 1,
                    &AutopilotMonitor::thrustCommandCallback, this);

  pixhawk_mode_sub_ = nh_.subscribe("/mavros/state", 1,
                                    &AutopilotMonitor::pixhawkModeCallback, this);

  rc_sub_ = nh_.subscribe("autopilot/rc/feedback", 1,
                              &AutopilotMonitor::rcCallback, this);

  point_sub_ =
      nh_.subscribe("/yolov5/gate/point", 1, &AutopilotMonitor::pointCallback, this);

  // mavros_pos_setpoint_sub_ =
  //     nh_.subscribe("/mavros/setpoint_position/local", 1, &AutopilotMonitor::positionCommandCallback, this);



}

void AutopilotMonitor::startServices() {
  server_ = new dynamic_reconfigure::Server<autopilot_monitor::BasicTuneConfig>(
      config_mutex_, ros::NodeHandle("~"));
  dynamic_reconfigure::Server<autopilot_monitor::BasicTuneConfig>::CallbackType f;
  f = boost::bind(&AutopilotMonitor::dynamicReconfigureCallback, this, _1, _2);
  server_->setCallback(f);
}

void AutopilotMonitor::startNode() {
  ros::TimerOptions timer_pub_options(
      ros::Duration(1.0 / kPubLoopFrequency_),
      boost::bind(&AutopilotMonitor::pubLoopCallback, this, _1),
      &pub_loop_queue_);
  pub_loop_timer_ = nh_.createTimer(timer_pub_options);

  pub_loop_spinner_.reset(new ros::AsyncSpinner(1, &pub_loop_queue_));
  pub_loop_spinner_->start();
}

bool AutopilotMonitor::loadParameters() {
  if (should_exit_) {
    return false;
  }
  ROS_INFO("-----------Loading AutopilotMonitor Parameters-----------");

#define GET_PARAM(name) \
  if (!vision_common::getParam(#name, name, pnh_)) return false
#define GET_PARAM_(name) \
  if (!vision_common::getParam(#name, name##_, pnh_)) return false

  GET_PARAM_(max_distance);
  GET_PARAM_(min_distance);

#undef GET_PARAM
#undef GET_PARAM_

  return true;
}


void AutopilotMonitor::pubLoopCallback(const ros::TimerEvent& event) {
  if (should_exit_) {
    return;
  }

  std::lock_guard<std::mutex> main_lock(main_mutex_);

  counter++;
  std::cout << "==================" << counter << "==================" << std::endl;
  std::cout.precision(3);

  if (autopilot_feedback_received_) {
    parseAutopilotSettings(feedback_msg_);
    parseStateAndReference(feedback_msg_.state_estimate, feedback_msg_.reference_state);
    parseControlCommand(feedback_msg_.control_command);
    // autopilot_feedback_received_ = false;
  }

  if (pixhawk_mode_received_) {
    parsePixhawkMode(pixhawk_mode_msg_);
    // pixhawk_mode_received_ = false;
  }

  if (rc_received_) {
    parseRcFeedback(rc_msg_);
    // rc_received_ = false;
  }

  if (point_available_) {
    parsePoints();
    point_available_ = false;
  }

  std::cout << std::endl;
}

void AutopilotMonitor::parseAutopilotSettings(
    const vision_msgs::AutopilotFeedback& msg) {
  std::cout << "(1) autopilot state:" << std::endl;
  std::string state_name;
  switch (msg.autopilot_state) {
    case msg.UNINIT:
      state_name = "UNINIT";
      break;
    case msg.BOOT:
      state_name = "BOOT";
      break;
    case msg.CALIBRATING:
      state_name = "CALIBRATING";
      break;
    case msg.STANDBY:
      state_name = "STANDBY";
      break;
    case msg.ACTIVE:
      state_name = "ACTIVE";
      break;
    case msg.CRITICAL:
      state_name = "CRITICAL";
      break;
    case msg.EMERGENCY:
      state_name = "EMERGENCY";
      break;
    case msg.POWEROFF:
      state_name = "POWEROFF";
      break;
    case msg.FLIGHT_TERMINATION:
      state_name = "FLIGHT_TERMINATION";
      break;
  }
  std::cout << state_name.c_str() << '\n' << std::endl;

  std::cout << "(2) control mode:" << std::endl;
  std::string control_mode_name;
  switch (msg.control_mode) {
    case msg.PX4_BODYRATES:
      control_mode_name = "PX4_BODYRATES";
      break;
    case msg.PX4_POSITION:
      control_mode_name = "PX4_POSITION";
      break;
    case msg.FSC_POSITION:
      control_mode_name = "FSC_POSITION";
      break;
  }

  std::cout << control_mode_name.c_str() << '\n' << std::endl;

  if (control_mode_name == "FSC_POSITION") {
    std::cout << "MPC solver:" << std::endl;
    std::cout << "Computational Time: " << msg.control_computation_time.toSec()*1000 << " ms" << std::endl;
    std::cout << "Computational Delay: " << msg.control_command_delay.toSec()*1000 << " ms" << std::endl;
    std::cout << "MPC success: " << (int)msg.mpc_success << std::endl;
    std::cout << std::endl;

    if (mavros_att_setpoint_received_) {
      std::cout << "Thrust Normalized: " << normalized_thrust_ << std::endl;
      std::cout << std::endl;
      mavros_att_setpoint_received_ = false;
    }
  }

  if (control_mode_name == "PX4_BODYRATES" && mavros_att_setpoint_received_) {
    std::cout << "Thrust Normalized: " << normalized_thrust_ << std::endl;
    std::cout << std::endl;
    mavros_att_setpoint_received_ = false;
  }

  std::cout << "(3) flight mode:" << std::endl;
  std::string flight_mode_name;
  switch (msg.flight_mode) {
    case msg.POSITION_TRACK:
      flight_mode_name = "POSITION_TRACK";
      break;
    case msg.TRAJECTORY_TRACK:
      flight_mode_name = "TRAJECTORY_TRACK";
      break;
    case msg.VELOCITY_TRACK:
      flight_mode_name = "VELOCITY_TRACK";
      break;
    case msg.FEATURE_TRACK:
      flight_mode_name = "FEATURE_TRACK";
      break;
    case msg.FEATURE_REACH:
      flight_mode_name = "FEATURE_REACH";
      break;
    case msg.HOME:
      flight_mode_name = "HOME";
      break;
    case msg.LAND:
      flight_mode_name = "LAND";
      break;
  }
  std::cout << flight_mode_name.c_str() << '\n' << std::endl;

  if (last_flight_mode_name_ != flight_mode_name) {
    max_speed_ = 0.0;
    last_flight_mode_name_ = flight_mode_name;
  }

}

void AutopilotMonitor::parseStateEstimate(
    const vision_msgs::StateEstimate& state_estimate) {
  vision_common::StateEstimate state(state_estimate);
  double heading = vision_common::quaternionToEulerAnglesZYX(state.orientation).z();
  std::cout << "(4) pose estimate:" << std::endl;
  std::cout << "Position: " << state.position.transpose() << std::endl;
  std::cout << "Heading: " << vision_common::rad2deg(heading) << " degree" << std::endl;
  std::cout << std::endl;

  if (state.velocity.norm() > max_speed_) {
    max_speed_ = state.velocity.norm();
  }

  std::cout << "Max speed: " << max_speed_ << " m/s" << std::endl;
  std::cout << std::endl;
}

void AutopilotMonitor::parseReferenceState(
    const vision_msgs::TrajectoryPoint& reference_state) {
  vision_common::TrajectoryPoint reference(reference_state);
  double heading = vision_common::quaternionToEulerAnglesZYX(reference.orientation).z();
  std::cout << "(5) reference pose:" << std::endl;
  std::cout << "Ref Pos: " << reference.position.transpose() << std::endl;
  std::cout << "Ref Heading: " << vision_common::rad2deg(heading) << " degree" << std::endl;
  std::cout << std::endl;
}

void AutopilotMonitor::parseStateAndReference(
    const vision_msgs::StateEstimate& state_estimate,
    const vision_msgs::TrajectoryPoint& reference_state) {
  vision_common::StateEstimate state(state_estimate);
  double heading = vision_common::quaternionToEulerAnglesZYX(state.orientation).z();
  std::cout << "(4) pose estimate:" << std::endl;
  std::cout << "Position: " << state.position.transpose() << std::endl;
  std::cout << "Heading: " << vision_common::rad2deg(heading) << " degree" << std::endl;
  std::cout << std::endl;

  if (state.velocity.norm() > max_speed_) {
    max_speed_ = state.velocity.norm();
  }

  std::cout << "Max speed: " << max_speed_ << " m/s" << std::endl;
  std::cout << std::endl;


  vision_common::TrajectoryPoint reference(reference_state);
  heading = vision_common::quaternionToEulerAnglesZYX(reference.orientation).z();
  std::cout << "(5) reference pose:" << std::endl;
  std::cout << "Ref Pos: " << reference.position.transpose() << std::endl;
  std::cout << "Ref Heading: " << vision_common::rad2deg(heading) << " degree" << std::endl;
  std::cout << std::endl;

  std::cout << "XY Error: " << (reference.position.head<2>() - state.position.head<2>()).norm() << " m" << std::endl;
  std::cout << "Z  Error: " << std::fabs(reference.position.z() - state.position.z()) << " m" << std::endl;
  std::cout << std::endl;
}

void AutopilotMonitor::parseControlCommand(
                          const vision_msgs::ControlCommand& control_command) {
  vision_common::ControlCommand command(control_command);
  std::cout << "(6) control commmand:" << std::endl;
  std::cout << "Thrust: " << command.collective_thrust << std::endl;
  std::cout << "Thrust Rate: " << command.thrust_rate << std::endl;
  std::cout << "Bodyreate: " << command.bodyrates.transpose() << std::endl;
  std::cout << std::endl;
}

void AutopilotMonitor::parsePixhawkMode(const mavros_msgs::State& msg) {
  std::cout << "(7) Pixhawk mode:" << std::endl;

  std::string mode_name = msg.mode ;
  std::cout << "PX4: " << mode_name.c_str() << std::endl;

  if (msg.armed) {
    std::cout << "PX4: Armed" << std::endl;
  } else {
    std::cout << "PX4: Not armed" << std::endl;
  }
  std::cout << std::endl;

}

void AutopilotMonitor::parseRcFeedback(const vision_msgs::RcFeedback& msg) {
  std::cout << "(8) RC output:" << std::endl;

  // msg.thrust;
  // msg.yaw;
  // msg.pitch;
  // msg.roll;
  // msg.scale1;
  // msg.scale2;
  scale1_ =  msg.scale1;
  scale2_ =  msg.scale2;

  std::cout << "Scale1: " << msg.scale1 << std::endl;
  std::cout << "Scale2: " << msg.scale2 << std::endl;

  std::cout << std::endl;
}

void AutopilotMonitor::parsePoints() {
  std::cout << "(9) FEATURE_TRACK ready:" << std::endl;
 
  std::cout << "Point in Cam: " << point_.toPoint().transpose() << std::endl;
  std::cout << "Global position: " << global_point_.transpose() << std::endl;
  std::cout << std::endl;

  double u = point_.toPoint().x() / point_.toPoint().z();
  double v = point_.toPoint().y() / point_.toPoint().z();

  if (std::fabs(u) < image_u_bound_ && std::fabs(v) < image_v_bound_) {
    std::cout << "Within Safe FOV (0.48, 0.25)." << std::endl;
    std::cout << std::endl;
  } else {
    std::cout << "Outside Safe FOV (0.48, 0.25)." << std::endl;
    std::cout << std::endl;
  }

  std::cout << "u: " << u << std::endl;
  std::cout << "v: " << v << std::endl;
  std::cout << std::endl;

  double reference_distance = scale2_ * (max_distance_ - min_distance_) + min_distance_;
  std::cout << "Reference_distance: " << reference_distance << " m" << std::endl;
  std::cout << "Distance: " << point_.distance << " m"<< std::endl;

  std::cout << std::endl;

}

void AutopilotMonitor::autopilotFeedbackCallback(
    const vision_msgs::AutopilotFeedback& msg) {
  if (should_exit_) {
    return;
  }

  std::lock_guard<std::mutex> main_lock(main_mutex_);
  feedback_msg_ = msg;

  autopilot_feedback_received_ = true;
}

void AutopilotMonitor::thrustCommandCallback(
    const mavros_msgs::AttitudeTarget& msg) {
  if (should_exit_) {
    return;
  }

  std::lock_guard<std::mutex> main_lock(main_mutex_);

  normalized_thrust_ = msg.thrust;
  mavros_att_setpoint_received_ = true;
};

void AutopilotMonitor::pixhawkModeCallback(const mavros_msgs::State& msg) {
  if (should_exit_) {
    return;
  }

  std::lock_guard<std::mutex> main_lock(main_mutex_);
  pixhawk_mode_msg_ = msg;
  pixhawk_mode_received_ = true;
}

void AutopilotMonitor::rcCallback(const vision_msgs::RcFeedback& msg) {
  if (should_exit_) {
    return;
  }

  std::lock_guard<std::mutex> main_lock(main_mutex_);

  rc_msg_= msg;
  rc_received_ = true;
}

void AutopilotMonitor::pointCallback(
    const sensor_msgs::PointCloud& msg) {
  if (should_exit_) {
    return;
  }

  std::lock_guard<std::mutex> main_lock(main_mutex_);

  if (msg.points.size() != 2) {
    return;
  }

  geometry_msgs::Point32 point = msg.points[0];
  geometry_msgs::Point32 global_point = msg.points[1];

  point_ = vision_common::VisualFeature(Eigen::Vector3d(
                                          point.x,
                                          point.y,
                                          point.z));

  global_point_ = Eigen::Vector3d(global_point.x, global_point.y, global_point.z);

  point_available_ = true;                           
}


void AutopilotMonitor::optitrackCallback(
    const geometry_msgs::PoseStamped& msg) {
  if (should_exit_) {
    return;
  }

  std::lock_guard<std::mutex> main_lock(main_mutex_);
  optitrack_received_ = true;
}


void AutopilotMonitor::dynamicReconfigureCallback(
    autopilot_monitor::BasicTuneConfig& config, uint32_t level) {
  if (should_exit_) {
    return;
  }

  ROS_INFO("Receive New Config!");
  rqt_param_config_ = config;

}

}  // namespace autopilot_monitor

