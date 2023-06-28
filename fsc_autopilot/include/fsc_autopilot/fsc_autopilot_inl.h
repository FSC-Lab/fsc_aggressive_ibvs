#pragma once

namespace fsc {

template <typename Tsystem, typename Tcontroller, typename Tparams>
Autopilot<Tsystem, Tcontroller, Tparams>::Autopilot(const ros::NodeHandle& nh,
                                       const ros::NodeHandle& pnh)
    : nh_(nh),
      pnh_(pnh),
      state_predictor_(nh_, pnh_),  
      should_exit_(false),
      odometry_available_(false),
      synchron_data_available_(false),
      command_available_(false),
      first_time_in_new_flight_mode_(false),
      velocity_estimate_in_world_frame_(false),
      enable_trajectory_replan_(false),
      enable_state_prediction_(false),
      enable_moving_target_track_(false),
      enable_rc_tune_(false),
      print_info_(false),
      emergency_land_thrust_(0.0),
      max_emergency_pos_z_(10.0),
      min_emergency_pos_z_(-1.0),
      max_emergency_pos_xy_(1.5),
      min_emergency_pos_xy_(-1.5),
      reference_thrust_(0.0),
      max_reference_pos_z_(10.0),
      min_reference_pos_z_(-1.0),
      max_reference_pos_xy_(1.5),
      min_reference_pos_xy_(-1.5),
      max_thrust_ratio_(1.0),
      min_thrust_ratio_(0.0),
      command_publish_frequency_(20.0),
      watchdog_frequency_(20.0),
      odometry_estimate_timeout_(0.1),
      synchron_data_timeout_(0.3),
      command_update_timeout_(0.3),
      emergency_landing_timeout_(5.0),
      flight_termination_timeout_(20.0),
      land_timeout_(5.0),
      home_timeout_(5.0),
      quad_radius_(0.1),
      computation_time_(1e-3),
      state_est_(),
      t_B_C_(Eigen::Vector3d::Zero()),
      q_B_C_(Eigen::Quaterniond::Identity()),
      last_heading_est_(0.0 * M_PI / 180.0),
      control_mode_(ControlMode::PX4_BODYRATES),
      flight_mode_(FlightMode::POSITION_TRACK),
      autopilot_state_(AutopilotState::UNINIT) {
  init();
}

template <typename Tsystem, typename Tcontroller, typename Tparams>
Autopilot<Tsystem, Tcontroller, Tparams>::~Autopilot() {
  should_exit_ = true;

  // stop the sensor synchronization thread
  {
    std::lock_guard<std::mutex> sensorsyn_lock(sensorsyn_mutex_);
    sensorsyn_cv_.notify_all();
  }

  if (sensorsyn_thread_.joinable()) sensorsyn_thread_.join();
  if (watchdog_thread_.joinable()) watchdog_thread_.join();
}

template <typename Tsystem, typename Tcontroller, typename Tparams>
void Autopilot<Tsystem, Tcontroller, Tparams>::init() {
  ROS_INFO("Initialize FSC Autopilot.");

  system_.init(nh_, pnh_);

  if (!loadParameters()) {
    ROS_ERROR("[%s] Could not load FSC_AUTOPILOT parameters.", pnh_.getNamespace().c_str());
    ros::shutdown();
    return;
  }

  // initialize standard subscribers
  odom_sub_ =
      nh_.subscribe("odom_topic", 1, &Autopilot::odometryCallback, this);

  point_sub_ =
      nh_.subscribe("point_topic", 1, &Autopilot::pointCallback, this);
  
  params_sub_ = nh_.subscribe("autopilot/tune_data_48", 1,
                              &Autopilot::paramsCallback, this);

  rc_sub_ = nh_.subscribe("autopilot/rc/feedback", 1,
                              &Autopilot::rcCallback, this);

  reference_pose_sub_ = nh_.subscribe(
      "autopilot/reference_pose", 1, &Autopilot::referencePoseCallback, this);

  reference_trajectory_sub_ = nh_.subscribe(
      "autopilot/reference_trajectory", 1, &Autopilot::referenceTrajectoryCallback, this);

  flight_mode_sub_ = nh_.subscribe(
      "autopilot/flight_mode", 1, &Autopilot::flightModeCallback, this);

  autopilot_feedback_pub_ =
      nh_.advertise<vision_msgs::AutopilotFeedback>("autopilot/feedback", 1);

  vis_drone_pub_ = nh_.advertise<visualization_msgs::Marker>("autopilot/vis_drone", 1);

  vis_goal_pub_ = nh_.advertise<visualization_msgs::Marker>("autopilot/vis_goal", 1);

  vis_trajectory_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
      "autopilot/vis_trajectory", 1);

  vis_dots_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
      "autopilot/mpcc_dots", 1);

  setAutopilotState(AutopilotState::BOOT);
  setFlightMode(FlightMode::POSITION_TRACK);

  startLoop();
  startThreads();
}

template <typename Tsystem, typename Tcontroller, typename Tparams>
void Autopilot<Tsystem, Tcontroller, Tparams>::reset() {
  synchron_data_available_ = false;
  odometry_available_ = false;
  command_available_ = false;
  command_queue_.clear();
  command_ = vision_common::ControlCommand();

  if (!loadParameters()) {
    ROS_ERROR("[%s] Could not load FSC_AUTOPILOT parameters.", pnh_.getNamespace().c_str());
    ros::shutdown();
    return;
  }

  setAutopilotState(AutopilotState::BOOT);
  setControlMode(ControlMode::PX4_BODYRATES);  
}

template <typename Tsystem, typename Tcontroller, typename Tparams>
void Autopilot<Tsystem, Tcontroller, Tparams>::startLoop() {
  // start command publication timer
  ros::TimerOptions timer_pub_options(
      ros::Duration(1.0 / command_publish_frequency_),
      boost::bind(&Autopilot<Tsystem, Tcontroller, Tparams>::commandLoopCallback, this, _1),
      &command_loop_queue_);
  command_loop_timer_ = nh_.createTimer(timer_pub_options);
  command_loop_spinner_.reset(new ros::AsyncSpinner(1, &command_loop_queue_));
  command_loop_spinner_->start();
}

template <typename Tsystem, typename Tcontroller, typename Tparams>
void Autopilot<Tsystem, Tcontroller, Tparams>::startThreads() {
  // start sensor synchronization thread
  try {
    sensorsyn_thread_ =
        std::thread(&Autopilot<Tsystem, Tcontroller, Tparams>::sensorSynchronizationThread, this);
  } catch (...) {
    ROS_ERROR(
        "[%s] Could not successfully start sensor synchronization thread.",
        pnh_.getNamespace().c_str());
    ros::shutdown();
    return;
  }

  // start watchdog thread
  try {
    watchdog_thread_ = std::thread(&Autopilot<Tsystem, Tcontroller, Tparams>::watchdogThread, this);
  } catch (...) {
    ROS_ERROR("[%s] Could not successfully start watchdog thread.",
              pnh_.getNamespace().c_str());
    ros::shutdown();
    return;
  }
}

template <typename Tsystem, typename Tcontroller, typename Tparams>
bool Autopilot<Tsystem, Tcontroller, Tparams>::armed() {
  return system_.armed();
}

template <typename Tsystem, typename Tcontroller, typename Tparams>
bool Autopilot<Tsystem, Tcontroller, Tparams>::ready() {
  return system_.ready();
}

template <typename Tsystem, typename Tcontroller, typename Tparams>
void Autopilot<Tsystem, Tcontroller, Tparams>::setAutopilotState(const AutopilotState& state) {
  autopilot_state_ = state;

  std::string state_name;
  switch (state) {
    case AutopilotState::UNINIT:
      state_name = "UNINIT";
      break;
    case AutopilotState::BOOT:
      state_name = "BOOT";
      break;
    case AutopilotState::CALIBRATING:
      state_name = "CALIBRATING";
      break;
    case AutopilotState::STANDBY:
      state_name = "STANDBY";
      break;
    case AutopilotState::ACTIVE:
      state_name = "ACTIVE";
      break;
    case AutopilotState::CRITICAL:
      state_name = "CRITICAL";
      break;
    case AutopilotState::EMERGENCY:
      state_name = "EMERGENCY";
      break;
    case AutopilotState::POWEROFF:
      state_name = "POWEROFF";
      break;
    case AutopilotState::FLIGHT_TERMINATION:
      state_name = "FLIGHT_TERMINATION";
      break;
    default:
      autopilot_state_ = AutopilotState::FLIGHT_TERMINATION;
      time_start_flight_termination_ = ros::Time::now();
      state_name = "FLIGHT_TERMINATION";
      ROS_ERROR("[%s] Cannot set an unknown autopilot state.", pnh_.getNamespace().c_str());
      break;
  }

  system_.setAutopilotState((int)autopilot_state_);

  ROS_INFO("[%s] Switch to %s state", pnh_.getNamespace().c_str(),
           state_name.c_str());
}

template <typename Tsystem, typename Tcontroller, typename Tparams>
void Autopilot<Tsystem, Tcontroller, Tparams>::setControlMode(const ControlMode& mode) {
  control_mode_ = mode;
  command_available_ = false;
  command_queue_.clear();
  computation_time_ = 0.003;

  std::string mode_name;
  switch (mode) {
    case ControlMode::PX4_BODYRATES:
      mode_name = "PX4_BODYRATES";
      break;
    case ControlMode::PX4_POSITION:
      mode_name = "PX4_POSITION";
      break;
    case ControlMode::FSC_POSITION:
      mode_name = "FSC_POSITION";
      break;
    default:
      ROS_ERROR("[%s] Encounter nknown control mode. Switch to PX4_BODYRATES mode", pnh_.getNamespace().c_str());
      control_mode_ = ControlMode::PX4_BODYRATES;
      mode_name = "PX4_BODYRATES";
      break;
  }

  ROS_INFO("[%s] Switch to %s control mode", pnh_.getNamespace().c_str(),
           mode_name.c_str());
}

template <typename Tsystem, typename Tcontroller, typename Tparams>
void Autopilot<Tsystem, Tcontroller, Tparams>::setFlightMode(const FlightMode& mode) {
  flight_mode_ = mode;
  first_time_in_new_flight_mode_ = true;
  computation_time_ = 0.003;

  std::string mode_name;
  switch (mode) {
    case FlightMode::POSITION_TRACK:
      mode_name = "POSITION_TRACK";
      break;
    case FlightMode::TRAJECTORY_TRACK:
      mode_name = "TRAJECTORY_TRACK";
      break;
    case FlightMode::VELOCITY_TRACK:
      mode_name = "VELOCITY_TRACK";
      break;
    case FlightMode::FEATURE_TRACK:
      mode_name = "FEATURE_TRACK";
      break;
    case FlightMode::FEATURE_REACH:
      mode_name = "FEATURE_REACH";
      break;
    case FlightMode::HOME:
      mode_name = "HOME";
      break;
    case FlightMode::LAND:
      mode_name = "LAND";
      break;
    default:
      ROS_ERROR("[%s] Cannot set an unknown flight mode.", pnh_.getNamespace().c_str());
      flight_mode_ = FlightMode::LAND;
      mode_name = "LAND";
      break;
  }

  ROS_INFO("[%s] Switch to %s flight mode", pnh_.getNamespace().c_str(),
           mode_name.c_str());
}

template <typename Tsystem, typename Tcontroller, typename Tparams>
AutopilotState Autopilot<Tsystem, Tcontroller, Tparams>::getAutopilotState() {
  return autopilot_state_;
}

template <typename Tsystem, typename Tcontroller, typename Tparams>
ControlMode Autopilot<Tsystem, Tcontroller, Tparams>::getControlMode() {
  return control_mode_;
}

template <typename Tsystem, typename Tcontroller, typename Tparams>
FlightMode Autopilot<Tsystem, Tcontroller, Tparams>::getFlightMode() {
  return flight_mode_;
}

template <typename Tsystem, typename Tcontroller, typename Tparams>
void Autopilot<Tsystem, Tcontroller, Tparams>::commandLoopCallback(
    const ros::TimerEvent& event) {
  if (should_exit_) {
    return;
  }

  std::lock_guard<std::mutex> command_lock(command_mutex_);

  switch (getAutopilotState()) {
    case AutopilotState::UNINIT:
      ROS_WARN("AutopilotState::UNINIT");
      break;
    case AutopilotState::BOOT:
      ROS_INFO("AutopilotState::BOOT");
      break;
    case AutopilotState::STANDBY:
      ROS_INFO("AutopilotState::STANDBY");
      //TODO:CHANGE
      sendCommand();
      break;
    case AutopilotState::ACTIVE:
      sendCommand();
      break;
    case AutopilotState::EMERGENCY:
      ROS_INFO("AutopilotState::EMERGENCY");
      break;
    case AutopilotState::FLIGHT_TERMINATION:
      ROS_WARN("AutopilotState::FLIGHT_TERMINATION");
      break;
    default:
      ROS_ERROR("Unknown AutopilotState");
      break;
  }

  if ((ros::Time::now() - time_last_autopilot_feedback_published_) >=
      ros::Duration(0.5)) {
    publishAutopilotFeedback(
      vision_common::StateEstimate(),
      reference_state_,
      vision_common::ControlCommand(),
      ros::Duration(0.0),
      ros::Duration(0.0), false); 
  }
}

template <typename Tsystem, typename Tcontroller, typename Tparams>
void Autopilot<Tsystem, Tcontroller, Tparams>::odometryCallback(
    const nav_msgs::Odometry& msg) {
  if (should_exit_) {
    return;
  }

  std::lock(sensorsyn_mutex_, goal_mutex_);
  std::lock_guard<std::mutex> lk1(sensorsyn_mutex_, std::adopt_lock);
  std::lock_guard<std::mutex> lk2(goal_mutex_, std::adopt_lock);

  received_state_est_ = vision_common::StateEstimate(msg);
  last_heading_est_ = 
    vision_common::quaternionToEulerAnglesZYX(received_state_est_.orientation).z();
    
  if (!velocity_estimate_in_world_frame_) {
    received_state_est_.transformVelocityToWorldFrame();
  }

  if (received_state_est_.velocity.norm() > max_speed_) {
    max_speed_ = received_state_est_.velocity.norm();
  }
  // std::cout << "speed: " << received_state_est_.velocity.norm() << std::endl;
  // std::cout << "max speed: " << max_speed_ << std::endl;

  if (!checkStateEstimate(received_state_est_)) {
    if (getAutopilotState() != AutopilotState::EMERGENCY) {
      setAutopilotState(AutopilotState::EMERGENCY);
      time_start_emergency_landing_ = ros::Time::now();
    }
    return;
  }

  state_predictor_.updateWithStateEstimate(received_state_est_);

  predicted_state_ = received_state_est_;
  if (enable_state_prediction_ && getAutopilotState() == AutopilotState::ACTIVE) {
    ros::Time wall_time_now = ros::Time::now();
    ros::Time command_execution_time =
        wall_time_now + ros::Duration(control_command_delay_);
    predicted_state_ = getPredictedStateEstimate(command_execution_time);
  }

  odometry_available_ = true;
  time_last_odometry_received_ = ros::Time::now();
  sensorsyn_cv_.notify_all();
}

template <typename Tsystem, typename Tcontroller, typename Tparams>
void Autopilot<Tsystem, Tcontroller, Tparams>::pointCallback(
    const sensor_msgs::PointCloud& msg) {
  if (should_exit_) {
    return;
  }

  std::lock(sensorsyn_mutex_, goal_mutex_);
  std::lock_guard<std::mutex> lk1(sensorsyn_mutex_, std::adopt_lock);
  std::lock_guard<std::mutex> lk2(goal_mutex_, std::adopt_lock);

  // if (msg.points.size() != 2) {
  //   return;
  // }

  geometry_msgs::Point32 point = msg.points[0];
  geometry_msgs::Point32 global_point = msg.points[1];

  point_ = vision_common::VisualFeature(Eigen::Vector3d(
                                          point.x,
                                          point.y,
                                          point.z));

  
  global_point_ = Eigen::Vector3d(global_point.x, global_point.y, global_point.z);
  // std::cout << "----------------------------" << std::endl;
  // std::cout << "point: " << point_.toPoint().transpose() << std::endl;
  // std::cout << "global_point: " << global_point_.transpose() << std::endl;

  double u = std::fabs(point_.toPoint().x() / point_.toPoint().z());
  double v = std::fabs(point_.toPoint().y() / point_.toPoint().z());
  if (u > image_u_bound_ || v > image_v_bound_) {
    // ROS_ERROR("[%s] Feature is outside the image bound!!!", pnh_.getNamespace().c_str());
    // invisibility_counter_++;
    // std::cout << invisibility_counter_ << std::endl;
  }

  // point_counter_++;

  point_available_ = true;
  time_last_point_received_ = ros::Time::now();  
  sensorsyn_cv_.notify_all();                            
}

template <typename Tsystem, typename Tcontroller, typename Tparams>
void Autopilot<Tsystem, Tcontroller, Tparams>::paramsCallback(
    const vision_msgs::TuneData48& msg) {
  if (should_exit_) {
    return;
  }

  std::lock(command_mutex_, goal_mutex_);
  std::lock_guard<std::mutex> lk1(command_mutex_, std::adopt_lock);
  std::lock_guard<std::mutex> lk2(goal_mutex_, std::adopt_lock);

  if (msg.bdata1) {
    reference_state_.position.x() = std::max(min_reference_pos_xy_, std::min(max_reference_pos_xy_, (double)msg.fdata1));
    reference_state_.position.y() = std::max(min_reference_pos_xy_, std::min(max_reference_pos_xy_, (double)msg.fdata2));
    reference_state_.position.z() = std::max(min_reference_pos_z_, std::min(max_reference_pos_z_, (double)msg.fdata3));
    double reference_heading = vision_common::deg2rad(msg.fdata4);
    reference_state_.heading =
      last_heading_est_ + std::max(-M_PI*0.3, std::min(M_PI*0.3, vision_common::wrapAngleDifference(last_heading_est_, reference_heading)));
    reference_state_.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(
        vision_common::wrapMinusPiToPi(reference_state_.heading), Eigen::Vector3d::UnitZ()));   
    if (getFlightMode() != FlightMode::POSITION_TRACK) {
      setFlightMode(FlightMode::POSITION_TRACK);
    }
  }

  if (msg.bdata2) {
    // TODO: change feedthrough by RC
    reference_thrust_ = msg.fdata5;
    setControlMode(ControlMode::PX4_BODYRATES);
  }

  if (msg.bdata3) {
    setControlMode(ControlMode::PX4_POSITION);
  }

  if (msg.bdata4) {
    if (!mpc_forbiddened_)
      setControlMode(ControlMode::FSC_POSITION);
  }

  if (msg.bdata5 && getAutopilotState() != AutopilotState::EMERGENCY) {
    setAutopilotState(AutopilotState::EMERGENCY);
    time_start_emergency_landing_ = ros::Time::now();
  }

  if (msg.bdata6 && getAutopilotState() != AutopilotState::FLIGHT_TERMINATION) {
    setAutopilotState(AutopilotState::FLIGHT_TERMINATION);
    time_start_flight_termination_ = ros::Time::now();
  }

  if (msg.bdata7 && getAutopilotState() != AutopilotState::ACTIVE) {
    setAutopilotState(AutopilotState::BOOT);
  }

  if (msg.bdata8 && getFlightMode() != FlightMode::POSITION_TRACK) {
    setFlightMode(FlightMode::POSITION_TRACK);
  }

  // set HOME
  if (msg.bdata9 && getAutopilotState() == AutopilotState::ACTIVE) {
    setFlightMode(FlightMode::HOME);
  }

  // set LAND
  if (msg.bdata10) {
    setFlightMode(FlightMode::LAND);
  }

  // TODO: FEATURE_TRACK can be triggered if yolov5 target point is available
  if (msg.bdata11 && getFlightMode() != FlightMode::FEATURE_TRACK && checkFeatureTrack()) {
    max_speed_ = 0.0;
    setFlightMode(FlightMode::FEATURE_TRACK);
  }

  if (!enable_moving_target_track_) {
    invisibility_counter_ = 0;
    point_counter_ = 0;
    reference_distance_ = msg.fdata6;
  }

  if (msg.bdata12 && getFlightMode() != FlightMode::FEATURE_REACH && checkFeatureReach()) {
    setFlightMode(FlightMode::FEATURE_REACH);
  }

  // std::cout << "reference_distance_: " << reference_distance_ << std::endl;

}

template <typename Tsystem, typename Tcontroller, typename Tparams>
void Autopilot<Tsystem, Tcontroller, Tparams>::rcCallback(const vision_msgs::RcFeedback& msg) {
  if (should_exit_) {
    return;
  }

  std::lock(command_mutex_, goal_mutex_);
  std::lock_guard<std::mutex> lk1(command_mutex_, std::adopt_lock);
  std::lock_guard<std::mutex> lk2(goal_mutex_, std::adopt_lock);

  if (!enable_rc_tune_) {
    return;
  }
  
  if (getAutopilotState() != AutopilotState::ACTIVE &&
      getAutopilotState() != AutopilotState::STANDBY) {
    return;
  }
  


  switch (msg.control_mode) {
    case msg.PX4_BODYRATES:
      // TODO: transfer it to real thrust
      system_.setThrustRatio(msg.scale1 * (max_thrust_ratio_ - min_thrust_ratio_) + min_thrust_ratio_);
      reference_thrust_ = msg.thrust * 5.0f;
      // std::cout << "Set thrust ratio as: "  << msg.scale1 * (max_thrust_ratio_ - min_thrust_ratio_) + min_thrust_ratio_ << std::endl;
      // std::cout << "Set reference thrust as: "  << reference_thrust_ << std::endl;
      if (getControlMode() != ControlMode::PX4_BODYRATES && !mpc_forbiddened_)
        setControlMode(ControlMode::PX4_BODYRATES);
      break;
    case msg.PX4_POSITION:
      if (getControlMode() != ControlMode::PX4_POSITION) {
        setControlMode(ControlMode::PX4_POSITION);
      }
      if (getFlightMode() != FlightMode::POSITION_TRACK) {
        reference_state_.position = received_state_est_.position;
        reference_state_.velocity.setZero();
        setFlightMode(FlightMode::POSITION_TRACK);
      }

      break;
    case msg.FSC_POSITION:
      system_.setThrustRatio(msg.scale1 * (max_thrust_ratio_ - min_thrust_ratio_) + min_thrust_ratio_);
      if (getControlMode() != ControlMode::FSC_POSITION && !mpc_forbiddened_) {
        setControlMode(ControlMode::FSC_POSITION);
        if (getFlightMode() != FlightMode::POSITION_TRACK) {
          reference_state_.position = received_state_est_.position;
          reference_state_.velocity.setZero();
          setFlightMode(FlightMode::POSITION_TRACK);
        }
      }
      
      if (mpc_forbiddened_) {
        ROS_WARN("[%s] Cannot switch to MPC for being forbiddened.", pnh_.getNamespace().c_str());
      }

      break;
    default:
      ROS_ERROR("[%s] Unknown control mode.", pnh_.getNamespace().c_str());
      return;
  }

  // if (getFlightMode() == FlightMode::FEATURE_TRACK) {
  //   // std::cout << "reference_distance: " << reference_distance_ << std::endl;
  // }

  reference_distance_ = msg.scale2 * (max_distance_ - min_distance_) + min_distance_;

  // switch (msg.flight_mode) {
  //   case msg.POSITION_TRACK:
  //     setFlightMode(FlightMode::POSITION_TRACK);
  //     break;
  //   case msg.FEATURE_TRACK:
  //     setFlightMode(FlightMode::FEATURE_TRACK);
  //     break;
  //   default:
  //     ROS_ERROR("[%s] Unknown flight mode.", pnh_.getNamespace().c_str());
  //     return;
  // }

}

template <typename Tsystem, typename Tcontroller, typename Tparams>
void Autopilot<Tsystem, Tcontroller, Tparams>::referencePoseCallback(
    const geometry_msgs::PoseStamped& msg) {
  if (should_exit_) {
    return;
  }

  std::lock_guard<std::mutex> goal_lock(goal_mutex_);

  if (getControlMode() != ControlMode::PX4_POSITION &&
      getControlMode() != ControlMode::FSC_POSITION) {
    return;
  }

  if (getFlightMode() == FlightMode::POSITION_TRACK ||
      getFlightMode() == FlightMode::TRAJECTORY_TRACK) {
    Eigen::Vector3d xyz = vision_common::geometryToEigen(msg.pose.position);
    if (std::isnan(xyz.norm())) {
      ROS_WARN("[%s] Received invalid target point, will ignore this point", pnh_.getNamespace().c_str());
      return;
    }

    reference_state_.position.x() = std::max(min_reference_pos_xy_, std::min(max_reference_pos_xy_, xyz.x()));
    reference_state_.position.y() = std::max(min_reference_pos_xy_, std::min(max_reference_pos_xy_, xyz.y()));
    reference_state_.position.z() = std::max(min_reference_pos_z_, std::min(max_reference_pos_z_, xyz.z()));
    reference_state_.velocity.setZero();

    double reference_heading = 
      vision_common::quaternionToEulerAnglesZYX(vision_common::geometryToEigen(msg.pose.orientation)).z();
    reference_state_.heading =
      last_heading_est_ + std::max(-M_PI*0.5, std::min(M_PI*0.5, vision_common::wrapAngleDifference(last_heading_est_, reference_heading)));
    reference_state_.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(vision_common::wrapMinusPiToPi(reference_state_.heading), Eigen::Vector3d::UnitZ()));
  }

  if (getFlightMode() == FlightMode::FEATURE_TRACK) {
    double reference_heading = 
      vision_common::quaternionToEulerAnglesZYX(vision_common::geometryToEigen(msg.pose.orientation)).z();
    reference_state_.heading =
      last_heading_est_ + std::max(-M_PI*0.5, std::min(M_PI*0.5, vision_common::wrapAngleDifference(last_heading_est_, reference_heading)));
    reference_state_.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(vision_common::wrapMinusPiToPi(reference_state_.heading), Eigen::Vector3d::UnitZ())); 
  }
}

template <typename Tsystem, typename Tcontroller, typename Tparams>
void Autopilot<Tsystem, Tcontroller, Tparams>::referenceTrajectoryCallback(
    const vision_msgs::Trajectory& msg) {
  if (should_exit_) {
    return;
  }

  std::lock_guard<std::mutex> goal_lock(goal_mutex_);

  if (getControlMode() != ControlMode::PX4_POSITION &&
      getControlMode() != ControlMode::FSC_POSITION) {
    return;
  }

  if (getFlightMode() != FlightMode::POSITION_TRACK &&
      getFlightMode() != FlightMode::TRAJECTORY_TRACK) {
    return;
  }

  if (msg.type == msg.UNDEFINED || msg.points.size() == 0) {
    ROS_WARN("[%s] Received invalid trajectory, will ignore trajectory", pnh_.getNamespace().c_str());
    return;
  }

  std::cout << "Receive trajectory" << std::endl;

  // if (trajectory_queue_.empty()) {
  //   // Check if there is a jump in the beginning of the trajectory
  //   if ((reference_state_.position -
  //        vision_common::geometryToEigen(msg.points[0].pose.position))
  //           .norm() > 1.0) {
  //     ROS_WARN(
  //         "[%s] First received trajectory segment does not start at current "
  //         "position, will ignore trajectory",
  //         pnh_.getNamespace().c_str());
  //     return;
  //   }
  // } else {
  //   // Check that there is no jump from the last trajectory in the queue to the
  //   // newly received one
  //   if (!enable_trajectory_replan_ && 
  //       (trajectory_queue_.back().points.back().position -
  //        vision_common::geometryToEigen(msg.points.front().pose.position)).norm() > 1.0) {
  //     ROS_WARN(
  //         "[%s] Received trajectory has a too large jump from the last "
  //         "trajectory in the queue, will ignore trajectory",
  //         pnh_.getNamespace().c_str());
  //     return;
  //   }
  // }

  if (!enable_trajectory_replan_) {
    // execute trajectories in order
    trajectory_queue_.push_back(vision_common::Trajectory(msg));
    if (getFlightMode() != FlightMode::TRAJECTORY_TRACK) {
      setFlightMode(FlightMode::TRAJECTORY_TRACK);
    }
  } else {
    // execute the newest trajectory and clear previous trajectories
    trajectory_queue_.clear();
    trajectory_queue_.push_back(vision_common::Trajectory(msg));
    setFlightMode(FlightMode::TRAJECTORY_TRACK);
  }


  // trajectory_queue_.clear();

  // Display trajectory point
  // std::cout << "---------------------------------------" << std::endl;
  // vision_common::Trajectory traj = trajectory_queue_.front();
  // std::cout.precision(3);
  // auto it = traj.points.begin();
  // for (; it != traj.points.end(); ++it) {
  //   std::cout << "time: " << it->time_from_start.toSec() << ", position: " << it->position.transpose() << std::endl;
  // }
}


template <typename Tsystem, typename Tcontroller, typename Tparams>
void Autopilot<Tsystem, Tcontroller, Tparams>::flightModeCallback(
    const vision_msgs::FlightMode& msg) {
  if (should_exit_) {
    return;
  }

  std::lock(command_mutex_, goal_mutex_);
  std::lock_guard<std::mutex> lk1(command_mutex_, std::adopt_lock);
  std::lock_guard<std::mutex> lk2(goal_mutex_, std::adopt_lock);

  switch (msg.flight_mode) {
    case msg.POSITION_TRACK:
      if (getFlightMode() != FlightMode::POSITION_TRACK) {
        reference_state_.position = received_state_est_.position;
        reference_state_.velocity.setZero();
        setFlightMode(FlightMode::POSITION_TRACK);
      }
      break;
    case msg.TRAJECTORY_TRACK:
      ROS_WARN("[%s] Cannot switch to TRAJECTORY_TRACK without reference trajectory.", pnh_.getNamespace().c_str());
      break;
    case msg.VELOCITY_TRACK:
      ROS_WARN("[%s] Cannot switch to VELOCITY_TRACK.", pnh_.getNamespace().c_str());
      break;
    case msg.FEATURE_TRACK:
      if (getFlightMode() != FlightMode::FEATURE_TRACK && checkFeatureTrack()) {
        setFlightMode(FlightMode::FEATURE_TRACK);
      }
      break;
    case msg.FEATURE_REACH:
      if (getFlightMode() != FlightMode::FEATURE_REACH && checkFeatureReach()) {
        setFlightMode(FlightMode::FEATURE_REACH);
      }
      break;
    case msg.HOME:
      if (getFlightMode() != FlightMode::HOME) {
        setFlightMode(FlightMode::HOME);
      }
      break;
    case msg.LAND:
      if (getFlightMode() != FlightMode::LAND) {
        setFlightMode(FlightMode::LAND);
      }
      break;
  }
}

template <typename Tsystem, typename Tcontroller, typename Tparams>
void Autopilot<Tsystem, Tcontroller, Tparams>::sensorSynchronizationThread() {
  while (!should_exit_) {
    std::unique_lock<std::mutex> sensorsyn_lock(sensorsyn_mutex_);

    sensorsyn_cv_.wait(sensorsyn_lock, [&] { return getStateEstimate(); });
    time_last_synchron_data_received_ = ros::Time::now();
    synchron_data_available_ = true;
    sensorsyn_lock.unlock();

    {
      std::lock(command_mutex_, goal_mutex_);
      std::lock_guard<std::mutex> lk1(command_mutex_, std::adopt_lock);
      std::lock_guard<std::mutex> lk2(goal_mutex_, std::adopt_lock);

      if (!readyToSolve()) {
        publishAutopilotFeedback(
          state_est_,
          reference_state_,
          vision_common::ControlCommand(),
          ros::Duration(0.0),
          ros::Duration(0.0), false); 

        continue;
      }

      // --------------Tit--------------
      const clock_t start = clock();
      
      solveCommands();

      const clock_t end = clock();
      // --------------Tat--------------

      computation_time_ =
          0.5 * computation_time_ + 0.5 * double(end - start) / CLOCKS_PER_SEC;
      
      // std::cout << "computation_time: " << computation_time_*1000 << " ms" << std::endl;
      //TODO:CHANGE
      if (computation_success_) {
        publishAutopilotFeedback(
          state_est_,
          reference_state_,
          command_queue_.front(),
          ros::Duration(control_command_delay_),
          ros::Duration(computation_time_), true);
      } else {
        publishAutopilotFeedback(
          state_est_,
          reference_state_,
          vision_common::ControlCommand(),
          ros::Duration(control_command_delay_),
          ros::Duration(computation_time_), false); 
      }
    }
  }  // end while
}

template <typename Tsystem, typename Tcontroller, typename Tparams>
void Autopilot<Tsystem, Tcontroller, Tparams>::watchdogThread() {
  ros::Rate watchdog_rate(watchdog_frequency_);
  while (ros::ok() && !should_exit_) {
    watchdog_rate.sleep();

    std::lock(command_mutex_, sensorsyn_mutex_);
    std::lock_guard<std::mutex> lk1(command_mutex_, std::adopt_lock);
    std::lock_guard<std::mutex> lk2(sensorsyn_mutex_, std::adopt_lock);

    const ros::Time time_now = ros::Time::now();

    // sensor watchdog
    if (odometry_available_ && time_now - time_last_odometry_received_ >
                                   ros::Duration(odometry_estimate_timeout_)) {
      std::cout << "time gap: " << (time_now - time_last_odometry_received_).toSec() << std::endl;
      ROS_WARN("[%s] Lose position estimate.", pnh_.getNamespace().c_str());
      odometry_available_ = false;

      // ROS_WARN("[%s] Switch to EMERGENCY for losing odometry.",
      //         pnh_.getNamespace().c_str());
      // setAutopilotState(AutopilotState::EMERGENCY);
      // time_start_emergency_landing_ = time_now;
    }

    if (point_available_ && time_now - time_last_point_received_ >
                                   ros::Duration(point_estimate_timeout_)) {
      if (getFlightMode() == FlightMode::FEATURE_TRACK || getFlightMode() == FlightMode::FEATURE_REACH) {
        ROS_WARN("[%s] Lose visual point.", pnh_.getNamespace().c_str());
        
        // TODO: set current position as reference position
        if (getFlightMode() != FlightMode::POSITION_TRACK) {
          reference_state_.position = received_state_est_.position;
          reference_state_.velocity.setZero();
          setFlightMode(FlightMode::POSITION_TRACK);  
        }
      }
      point_available_ = false;
    }

    if (synchron_data_available_ && time_now - time_last_synchron_data_received_ >
            ros::Duration(synchron_data_timeout_)) {
      ROS_ERROR("[%s] Lose synchronized data.", pnh_.getNamespace().c_str());
      synchron_data_available_ = false;
    }

    // switch back to BOOT if (1) data lost
    if (getAutopilotState() == AutopilotState::ACTIVE) {
      if (!synchron_data_available_) {
        if (odometry_available_) {
          // TODO: since odometry is still available, we can try POSITION_TRACK
          ROS_WARN("[%s] Switch to POSITION_TRACK for losing synchronized data but odometry is still available.",
               pnh_.getNamespace().c_str());
          
          if (getFlightMode() != FlightMode::POSITION_TRACK) {
            reference_state_.position = received_state_est_.position;
            reference_state_.velocity.setZero();
            setFlightMode(FlightMode::POSITION_TRACK);  
          }
        } else{
          ROS_WARN("[%s] Switch to EMERGENCY for losing synchronized data and odometry.",
                  pnh_.getNamespace().c_str());
          setAutopilotState(AutopilotState::EMERGENCY);
          time_start_emergency_landing_ = time_now;
        }
      }
    }

    if (getControlMode() == ControlMode::FSC_POSITION &&
        getAutopilotState() == AutopilotState::ACTIVE && 
        command_available_ && 
        time_now - time_last_command_queue_updated_ > ros::Duration(command_update_timeout_)) {
      ROS_ERROR("[%s] Switch to PX4_POSITION mode for losing available MPC commands.",
               pnh_.getNamespace().c_str());
      command_queue_.clear();
      command_available_ = false;

      if (getFlightMode() != FlightMode::POSITION_TRACK) {
        reference_state_.position = received_state_est_.position;
        reference_state_.velocity.setZero();
        setFlightMode(FlightMode::POSITION_TRACK);  
      }
      setControlMode(ControlMode::PX4_POSITION);
      mpc_forbiddened_ = true;
    }

    // switch to STANDBY if (1) data ready
    if (getAutopilotState() == AutopilotState::BOOT) {
      if (synchron_data_available_) {
        ROS_INFO("[%s] Switch to STANDBY for having synchronized data.",
                 pnh_.getNamespace().c_str());
        setAutopilotState(AutopilotState::STANDBY);
      }
    }

    // switch back to BOOT if (1) data lost
    if (getAutopilotState() == AutopilotState::STANDBY) {
      if (!synchron_data_available_) {
        if (odometry_available_) {
          // TODO: since odometry is still available, we can try POSITION_TRACK
          ROS_WARN("[%s] Switch to POSITION_TRACK for losing synchronized data but odometry is still available.",
               pnh_.getNamespace().c_str());
          
          if (getFlightMode() != FlightMode::POSITION_TRACK) {
            setFlightMode(FlightMode::POSITION_TRACK);  
          }

        } else{
          ROS_ERROR("[%s] Switch to BOOT for losing synchronized data",
                    pnh_.getNamespace().c_str());
          setAutopilotState(AutopilotState::BOOT);
        }
      }
    }

    // switch to ACTIVE if (1) STANDBY; (2) OFFBOARD mode; (3) data ready
    if (getAutopilotState() == AutopilotState::STANDBY && ready()) {
      if (synchron_data_available_) {
        ROS_INFO("[%s] Switch to ACTIVE for entering OFFBOARD mode.",
                 pnh_.getNamespace().c_str());
        setAutopilotState(AutopilotState::ACTIVE);
      }
    }

    // switch to BOOT if (1) ACTIVE; (2) not OFFBOARD mode;
    if (getAutopilotState() == AutopilotState::ACTIVE && !ready()) {
      ROS_WARN("[%s] Switch to BOOT for leaviing OFFBOARD mode.",
               pnh_.getNamespace().c_str());
      setAutopilotState(AutopilotState::BOOT);
    }

    // switch to FLIGHT_TERMINATION if (1) stay at EMERGENCY for a while
    if (getAutopilotState() == AutopilotState::EMERGENCY) {
      if (time_now - time_start_emergency_landing_ >
          ros::Duration(emergency_landing_timeout_)) {
        ROS_INFO(
            "[%s] Switch to FLIGHT_TERMINATION for ending emergency "
            "land",
            pnh_.getNamespace().c_str());
        setAutopilotState(AutopilotState::FLIGHT_TERMINATION);
        time_start_flight_termination_ = time_now;
      } else {
        sendEmergencyCommand();
      }
    }

    // recover from FLIGHT_TERMINATION if (1) after a long time
    if (getAutopilotState() == AutopilotState::FLIGHT_TERMINATION) {
      if (time_now - time_start_flight_termination_ >
          ros::Duration(flight_termination_timeout_)) {
        ROS_INFO("[%s] Switch to BOOT for recovering flights.",
                 pnh_.getNamespace().c_str());
        reset();
      }
    }

  }  // end while
}

template <typename Tsystem, typename Tcontroller, typename Tparams>
void Autopilot<Tsystem, Tcontroller, Tparams>::sendCommand() {
  switch (getControlMode()) {
    case ControlMode::PX4_BODYRATES:
      // std::cout << "ControlMode::PX4_BODYRATES" << std::endl;
      sendBodyRates(Eigen::Vector3d::Zero(), reference_thrust_);
      return;
    case ControlMode::PX4_POSITION:
      // sendFirstCommand();
      // std::cout << "ControlMode::PX4_POSITION" << std::endl;
      sendPosition(reference_state_.position, reference_state_.orientation);
      return;
    case ControlMode::FSC_POSITION:
      // std::cout << "ControlMode::FSC_POSITION" << std::endl;
      sendFirstCommand();
      return;
    default:
      ROS_ERROR("[%s] Unknown control mode.", pnh_.getNamespace().c_str());
      setAutopilotState(AutopilotState::UNINIT);
      break;
  }
}

template <typename Tsystem, typename Tcontroller, typename Tparams>
void Autopilot<Tsystem, Tcontroller, Tparams>::sendFirstCommand() {
  if (command_queue_.empty() || !command_available_) return;

  command_ = command_queue_.front();
  command_queue_.pop_front();

  if (!validateCommand(command_)) {
    setAutopilotState(AutopilotState::EMERGENCY);
    time_start_emergency_landing_ = ros::Time::now();
    return;
  }

  if (print_info_) {
    std::cout << "-----------Command-----------" << std::endl;
    std::cout.precision(3);
    std::cout << "Thrust: " << command_.collective_thrust << std::endl;
    std::cout << "Thrust rate: " << command_.thrust_rate << std::endl;
    std::cout << "Bodyrates: " << command_.bodyrates.transpose() << std::endl;
  }

  // TODO: predict states using new command
  // state_predictor_.pushCommandToQueue(command_);

  sendBodyRates(command_);
}

template <typename Tsystem, typename Tcontroller, typename Tparams>
void Autopilot<Tsystem, Tcontroller, Tparams>::sendEmergencyCommand() {

  sendBodyRates(Eigen::Vector3d::Zero(), emergency_land_thrust_);

  vision_common::ControlCommand thrust_command;
  thrust_command.collective_thrust = emergency_land_thrust_;

  publishAutopilotFeedback(
    vision_common::StateEstimate(),
    vision_common::TrajectoryPoint(),
    thrust_command,
    ros::Duration(0.0),
    ros::Duration(0.0), false); 
}

template <typename Tsystem, typename Tcontroller, typename Tparams>
void Autopilot<Tsystem, Tcontroller, Tparams>::sendBodyRates(
    const Eigen::Vector3d& bodyrate,
    const double& thrust) {

  vision_common::ControlCommand command = vision_common::ControlCommand();
  command.timestamp = ros::Time::now();
  command.armed = true;
  command.control_mode = vision_common::ControlMode::BODY_RATES;
  command.expected_execution_time = ros::Time::now();
  command.collective_thrust = thrust;
  command.thrust_rate = 0.0;
  command.bodyrates.x() = bodyrate.x();
  command.bodyrates.y() = bodyrate.y();
  command.bodyrates.z() = bodyrate.z();
  command.orientation.w() = 1.0;
  command.orientation.x() = 0.0;
  command.orientation.y() = 0.0;
  command.orientation.z() = 0.0;

  sendBodyRates(command);

}

template <typename Tsystem, typename Tcontroller, typename Tparams>
void Autopilot<Tsystem, Tcontroller, Tparams>::sendBodyRates(const vision_common::ControlCommand& command) {
  system_.sendBodyRates(command);
}

template <typename Tsystem, typename Tcontroller, typename Tparams>
void Autopilot<Tsystem, Tcontroller, Tparams>::sendPosition(
    const Eigen::Vector3d& position,
    const Eigen::Quaterniond& orientation) {
  system_.sendPosition(position, orientation);
}

template <typename Tsystem, typename Tcontroller, typename Tparams>
void Autopilot<Tsystem, Tcontroller, Tparams>::solveCommands() {
  switch (getFlightMode()) {
    case FlightMode::POSITION_TRACK:
      trackPosition();
      break;
    case FlightMode::TRAJECTORY_TRACK:
      trackTrajectory();
      break;
    case FlightMode::VELOCITY_TRACK:
      trackVelocity();
      break;
    case FlightMode::FEATURE_TRACK:
      trackFeature();
      break;
    case FlightMode::FEATURE_REACH:
      // reachFeature();
      break;
    case FlightMode::HOME:
      home();
      break;
    case FlightMode::LAND:
      land();
      break;
    default:
      ROS_ERROR("[%s] Unknown flight mode.", pnh_.getNamespace().c_str());
      setAutopilotState(AutopilotState::FLIGHT_TERMINATION);
      time_start_flight_termination_ = ros::Time::now();
      break;
  }
}

template <typename Tsystem, typename Tcontroller, typename Tparams>
bool Autopilot<Tsystem, Tcontroller, Tparams>::validateCommand(
    const vision_common::ControlCommand& command) {
  bool valid = true;

  if (std::isnan(command.collective_thrust) ||
      std::isnan(command.thrust_rate) ||
      std::isnan(command.bodyrates.x()) ||
      std::isnan(command.bodyrates.y()) ||
      std::isnan(command.bodyrates.z())) {
    ROS_ERROR("[%s] Control inputs contain NaN.", pnh_.getNamespace().c_str());
    valid = false;
  }

  return valid;
}

template <typename Tsystem, typename Tcontroller, typename Tparams>
bool Autopilot<Tsystem, Tcontroller, Tparams>::readyToSolve() {
  bool valid = true;

  if (getAutopilotState() != AutopilotState::STANDBY &&
      getAutopilotState() != AutopilotState::ACTIVE) {
    valid = false;
  }

  // if (getControlMode() != ControlMode::FSC_POSITION) {
  //   valid = false;
  // }

  if (mpc_forbiddened_) {
    valid = false;
  }

  return valid;
}

template <typename Tsystem, typename Tcontroller, typename Tparams>
bool Autopilot<Tsystem, Tcontroller, Tparams>::checkStateEstimate(const vision_common::StateEstimate& state_est) {
  bool valid = true;

  if (!state_est.isValid()) {
    ROS_WARN("[%s] Invalid state estimate.", pnh_.getNamespace().c_str());
    valid = false;
  }

  Eigen::Vector3d euler_est = vision_common::quaternionToEulerAnglesZYX(state_est.orientation);
  euler_est.x() = vision_common::wrapMinusPiToPi(euler_est.x());
  euler_est.y() = vision_common::wrapMinusPiToPi(euler_est.y());

  if (fabs(euler_est.x()) >= M_PI*0.5 ||
      fabs(euler_est.y()) >= M_PI*0.5) {
    std::cout << "roll: " << vision_common::rad2deg(euler_est.x()) << std::endl;      
    std::cout << "pitch: " << vision_common::rad2deg(euler_est.y()) << std::endl;    
    std::cout << "yaw: " << vision_common::rad2deg(euler_est.z()) << std::endl;    

    ROS_WARN("[%s] Bad Euler angles.", pnh_.getNamespace().c_str());
    valid = false;
  }

  if (state_est.position.z() > max_emergency_pos_z_ || state_est.position.z() < min_emergency_pos_z_ ||
      state_est.position.x() > max_emergency_pos_xy_ || state_est.position.x() < min_emergency_pos_xy_ ||
      state_est.position.y() > max_emergency_pos_xy_ || state_est.position.y() < min_emergency_pos_xy_) {
    ROS_WARN("[%s] Out of safe regions.", pnh_.getNamespace().c_str());
    valid = false;
  }

  return valid;
}

template <typename Tsystem, typename Tcontroller, typename Tparams>
vision_common::StateEstimate Autopilot<Tsystem, Tcontroller, Tparams>::getPredictedStateEstimate(const ros::Time& time) {
  return state_predictor_.predictState(time);
}


template <typename Tsystem, typename Tcontroller, typename Tparams>
void Autopilot<Tsystem, Tcontroller, Tparams>::trackReferenceState(
    const vision_common::TrajectoryPoint& reference_state) {
  Tparams controller_params = base_controller_params_;
  
  vision_common::StateEstimate state_est;
  bool res_state = generateState(state_est, controller_params);
  if (!res_state) {
    return;
  }

  vision_common::Trajectory reference_trajectory;
  bool res_ref = generateReferenceTrajectory(reference_state, reference_trajectory, controller_params);
  if (!res_ref) {
    return;
  }

  std::list<vision_common::ControlCommand> command_queue;
  std::vector<vision_common::StateEstimate> predicted_states;

  bool res_mpc = base_controller_.run(state_est, reference_trajectory, controller_params, command_queue, predicted_states);
  computation_success_ = res_mpc;

  if (!res_mpc) {
    ROS_ERROR("[%s] Fail to solve mpc. Switch to PX4_POSITION", pnh_.getNamespace().c_str());
    command_available_ = false;
    command_queue_.clear();
    setControlMode(ControlMode::PX4_POSITION);
    mpc_forbiddened_ = true;
    // setAutopilotState(AutopilotState::EMERGENCY);
    // time_start_emergency_landing_ = ros::Time::now();
    return;
  }

  command_queue_ = command_queue;
  // TODO: remove the addition of thrust and thrust_dot * dt in mpc_control
  // command_queue_.pop_front();

  mpc_states_ = predicted_states;

  command_available_ = true;
  time_last_command_queue_updated_ = ros::Time::now();
}

template <typename Tsystem, typename Tcontroller, typename Tparams>
void Autopilot<Tsystem, Tcontroller, Tparams>::trackPosition() {
  const ros::Time time_now = ros::Time::now();
  if (first_time_in_new_flight_mode_) {
    first_time_in_new_flight_mode_ = false;
  }
  
  trackReferenceState(reference_state_);
}

template <typename Tsystem, typename Tcontroller, typename Tparams>
void Autopilot<Tsystem, Tcontroller, Tparams>::trackTrajectory() {
  const ros::Time time_now = ros::Time::now();
  // std::cout << "Tracking trajectory!" << std::endl;
  if (first_time_in_new_flight_mode_) {
    first_time_in_new_flight_mode_ = false;
    time_start_trajectory_track_ = time_now;
  }

  if (trajectory_queue_.empty()) {
    ROS_INFO("[%s] Trajectory queue was empty, going back to POSITION_TRACK",
             pnh_.getNamespace().c_str());
    setFlightMode(FlightMode::POSITION_TRACK);
    trackPosition();
    return;
  }

  if ((time_now - time_start_trajectory_track_) > trajectory_queue_.front().points.back().time_from_start) {
    if (trajectory_queue_.size() == 1) {
      // track the last point and return to POSITION_TRACK
      reference_state_ = trajectory_queue_.back().points.back();
      trajectory_queue_.pop_front();
      setFlightMode(FlightMode::POSITION_TRACK);
      trackPosition();
      return;
    } else {
      // switch to next trajectory
      time_start_trajectory_track_ += trajectory_queue_.front().points.back().time_from_start;
      trajectory_queue_.pop_front();
    }
  }

  // set state estimate
  Tparams controller_params = base_controller_params_;
  vision_common::StateEstimate state_est;
  bool res_state = generateState(state_est, controller_params);
  if (!res_state) {
    return;
  }

  const ros::Duration dt = time_now - time_start_trajectory_track_;

  // reference_state_ = trajectory_queue_.front().getStateAtTime(dt);

  // set reference trajectory
  vision_common::Trajectory reference_trajectory;
  bool res_ref = generateReferenceTrajectory(dt, reference_trajectory, controller_params);
  if (!res_ref) {
    return;
  }

  std::list<vision_common::ControlCommand> command_queue;
  std::vector<vision_common::StateEstimate> predicted_states;
  bool res_mpc = base_controller_.run(state_est, reference_trajectory, controller_params, command_queue, predicted_states);
  computation_success_ = res_mpc;

  if (!res_mpc) {
    ROS_ERROR("[%s] Fail to solve mpc.", pnh_.getNamespace().c_str());
    command_available_ = false;
    command_queue_.clear();
    setControlMode(ControlMode::PX4_POSITION);
    mpc_forbiddened_ = true;
    // setAutopilotState(AutopilotState::EMERGENCY);
    // time_start_emergency_landing_ = ros::Time::now();
    return;
  }

  command_queue_ = command_queue;
  // command_queue_.pop_front();

  mpc_states_ = predicted_states;

  command_available_ = true;
  time_last_command_queue_updated_ = ros::Time::now();
}

template <typename Tsystem, typename Tcontroller, typename Tparams>
void Autopilot<Tsystem, Tcontroller, Tparams>::trackVelocity() {
  const ros::Time time_now = ros::Time::now();
  if (first_time_in_new_flight_mode_) {
    first_time_in_new_flight_mode_ = false;
  }
}

template <typename Tsystem, typename Tcontroller, typename Tparams>
void Autopilot<Tsystem, Tcontroller, Tparams>::goMPC(const vision_common::StateEstimate& state_est) {
  // std::cout << "MPC Mode" << std::endl;
  reference_state_.position = computeReferencePosition(reference_distance_);

  Tparams controller_params = feature_track_params_;
  controller_params.setFocus(global_point_, Eigen::Quaterniond::Identity());
  controller_params.setReferencePoint(reference_state_.position);

  // TODO: plan a smooth trajectory
  vision_common::TrajectoryPoint start_state;
  start_state.position = state_est.position;
  start_state.heading = vision_common::quaternionToEulerAnglesZYX(state_est.orientation).z();

  reference_trajectory_ =
      trajectory_generation_helper::polynomials::computeTimeOptimalTrajectory(
          start_state, reference_state_, order_, max_planning_vel_, max_thrust_,
          max_roll_pitch_rate_, sample_rate_);
  trajectory_generation_helper::heading::addConstantHeadingRate(
      start_state.heading, reference_state_.heading, &reference_trajectory_);

  // reference_trajectory_ = vision_common::Trajectory(reference_state_);
  // reference_trajectory_.trajectory_type = vision_common::Trajectory::TrajectoryType::GENERAL;

  std::list<vision_common::ControlCommand> command_queue;

  bool res_mpc = base_controller_.run(state_est, reference_trajectory_, controller_params, command_queue);
  computation_success_ = res_mpc;

  if (!res_mpc) {
    ROS_ERROR("[%s] Fail to solve mpc. Switch to PX4_POSITION", pnh_.getNamespace().c_str());
    command_available_ = false;
    command_queue_.clear();
    setControlMode(ControlMode::PX4_POSITION);
    mpc_forbiddened_ = true;
    return;
  }

  command_queue_ = command_queue;
  // command_queue_.pop_front();

  command_available_ = true;
  time_last_command_queue_updated_ = ros::Time::now();
}

template <typename Tsystem, typename Tcontroller, typename Tparams>
bool Autopilot<Tsystem, Tcontroller, Tparams>::checkFeatureTrack() {
  if (!point_available_) {
    ROS_WARN("[%s] Cannot switch to FEATURE_TRACK for point unavailable,", pnh_.getNamespace().c_str());
    return false;
  }

  if (mpc_forbiddened_) {
    ROS_WARN("[%s] Cannot switch to FEATURE_TRACK for MPC forbiddened.,", pnh_.getNamespace().c_str());
    return false;
  }

  // if (std::fabs(reference_distance_ - point_.toPoint().norm()) > feature_track_max_distance_) {
  //   double error = std::fabs(reference_distance_ - point_.toPoint().norm());
  //   std::cout << "reference distance_: " << reference_distance_ << std::endl;
  //   std::cout << "current distance: " << point_.toPoint().norm() << std::endl;

  //   std::cout << "error: " << error << std::endl;
  //   ROS_WARN("[%s] Cannot switch to FEATURE_TRACK for large distance error.", pnh_.getNamespace().c_str());
  //   return false;
  // }

  if (point_.toPoint().z() < 1e-3) {
    ROS_WARN("[%s] Cannot switch to FEATURE_REACH for bad distance to the feature.", pnh_.getNamespace().c_str());
    return false;
  }

  double u = std::fabs(point_.toPoint().x() / point_.toPoint().z());
  double v = std::fabs(point_.toPoint().y() / point_.toPoint().z());
  if (u > image_u_bound_ || v > image_v_bound_) {
    ROS_WARN("[%s] Cannot switch to FEATURE_TRACK for features outside the perception awareness bounding box.", pnh_.getNamespace().c_str());
    return false;
  }

  return true;
}

template <typename Tsystem, typename Tcontroller, typename Tparams>
bool Autopilot<Tsystem, Tcontroller, Tparams>::checkFeatureReach() {

  if (getFlightMode() != FlightMode::FEATURE_TRACK) {
    ROS_WARN("[%s] Cannot switch to FEATURE_REACH for not starting from FEATURE_TRACK mode.", pnh_.getNamespace().c_str());
    return false;
  }

  return true;
}

template <typename Tsystem, typename Tcontroller, typename Tparams>
Eigen::Vector3d Autopilot<Tsystem, Tcontroller, Tparams>::computeReferencePosition(const double reference_distance) {
  Eigen::Vector3d P_W_LW = global_point_;
  // Eigen::Vector3d P_W_LW(20, 0, 3.0);
  Eigen::Quaterniond q_W_L = Eigen::Quaterniond::Identity();
  // reference relative pose in camera and body frames
  Eigen::Vector3d p_C_LC = Eigen::Vector3d(0, height_gap_, reference_distance);
  Eigen::Vector3d p_B_LB = q_B_C_ * p_C_LC + t_B_C_;
  // reference heading
  Eigen::Quaterniond q_W_B = reference_state_.orientation;
  // compute corresponding global reference position
  Eigen::Vector3d p_W_BL = -(q_W_B * p_B_LB);
  Eigen::Vector3d p_W_BW = p_W_BL + P_W_LW;

  // std::cout << "----------------" << std::endl;
  // std::cout << "current distance: " << point_.toPoint().norm() << std::endl;
  // std::cout << "reference distance: " << reference_distance << std::endl;
  // std::cout << std::endl;
  // std::cout << "gate position: " << global_point_.transpose() << std::endl;
  // std::cout << std::endl;
  // std::cout << "current drone position: " << state_est_.position.transpose() << std::endl;
  // std::cout << "target drone position: " << p_W_BW.transpose() << std::endl;

  return p_W_BW;
}

template <typename Tsystem, typename Tcontroller, typename Tparams>
void Autopilot<Tsystem, Tcontroller, Tparams>::trackFeature() {
  const ros::Time time_now = ros::Time::now();
  if (first_time_in_new_flight_mode_) {
    first_time_in_new_flight_mode_ = false;
  }

  vision_common::StateEstimate state_est;
  Tparams controller_params;
  bool res_state = generateState(state_est, controller_params);
  if (!res_state) {
    return;
  }

  /**********************************/
  reference_state_.position = computeReferencePosition(reference_distance_);

  controller_params = feature_track_params_;
  controller_params.setFocus(global_point_, Eigen::Quaterniond::Identity());
  // controller_params.setReferencePoint(reference_state_.position);

  double current_distance = state_est.gates.gates[0].center.distance;
  double distance_gap = std::fabs(reference_distance_ - current_distance);
  if (distance_gap > feature_track_max_distance_) {
    // TODO: plan a smooth trajectory
    vision_common::TrajectoryPoint start_state;
    start_state.position = state_est.position;
    start_state.velocity = state_est.velocity;
    start_state.heading = vision_common::quaternionToEulerAnglesZYX(state_est.orientation).z();

    // plane a trajectory
    // reference_trajectory_ =
    //     trajectory_generation_helper::polynomials::computeTimeOptimalTrajectory(
    //         start_state, reference_state_, order_, max_planning_vel_, max_thrust_,
    //         max_roll_pitch_rate_, sample_rate_);
    // trajectory_generation_helper::heading::addConstantHeadingRate(
    //     start_state.heading, reference_state_.heading, &reference_trajectory_);

    std::vector<Eigen::Vector3d> way_points;
    Eigen::VectorXd segment_times =
        Eigen::VectorXd::Ones(int(way_points.size()) + 1);

    polynomial_trajectories::PolynomialTrajectorySettings trajectory_settings;
    Eigen::VectorXd minimization_weights(5);  // pos, vel, acc, jerk, snap
    minimization_weights << 0.0, 1.0, 1.0, 0.0, 0.0;
    trajectory_settings.minimization_weights = minimization_weights;
    trajectory_settings.continuity_order = 3;
    trajectory_settings.polynomial_order = 7;
    trajectory_settings.way_points = way_points;

    vision_common::TrajectoryPoint goal_state = reference_state_;    
    reference_trajectory_ = trajectory_generation_helper::polynomials::generateMinimumSnapTrajectory(
            segment_times, start_state, goal_state, trajectory_settings,
            max_planning_vel_, max_thrust_, max_roll_pitch_rate_, sample_rate_);

    if (reference_trajectory_.trajectory_type != vision_common::Trajectory::TrajectoryType::GENERAL) {
      ROS_WARN("[%s] Feature track planning fails.", pnh_.getNamespace().c_str());
      reference_state_.position = state_est.position;
      reference_state_.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(vision_common::quaternionToEulerAnglesZYX(state_est.orientation).z(), Eigen::Vector3d::UnitZ()));
      reference_state_.velocity.setZero();
      setFlightMode(FlightMode::POSITION_TRACK);
      trackPosition();
      return;  
    }

    trajectory_generation_helper::heading::addConstantHeadingRate(
        start_state.heading, goal_state.heading, &reference_trajectory_);


  } else {
    // directly track the target
    reference_trajectory_ = vision_common::Trajectory(reference_state_);
    reference_trajectory_.trajectory_type = vision_common::Trajectory::TrajectoryType::GENERAL;
  }

  // std::cout << "point_counter: " << point_counter_ << std::endl;
  // std::cout << "distance_gap: " << distance_gap << std::endl;

  if (distance_gap < 0.2 && use_flip_mode_) {


    // std::cout << "invisibiliy rate: " << double(invisibility_counter_) / double(point_counter_) * 100 << " %" << std::endl;
    if (flip_) {
      if (point_counter_ == 10) {
        std::cout << "STOP RECORDING!" << std::endl;
        use_flip_mode_ = false;
      }
      reference_distance_ = 20.0;
      flip_ = false;
    } else {
      std::cout << "=======================" << std::endl;
      std::cout << "round: " << point_counter_ << std::endl;
      std::cout << "max speed: " << max_speed_ << std::endl;

      point_counter_++;
      reference_distance_ = 3.0;
      flip_ = true;
    }
    
  }

  //TODO: plot the temporal reference trajectory for MPC
  while (!reference_trajectory_.points.empty() && reference_trajectory_.points.size() > 21) {
    reference_trajectory_.points.pop_back();
  }
  vis_trajectory_pub_.publish(makeMarkerArray(reference_trajectory_, Eigen::Vector3d(0,0,1)));


  std::list<vision_common::ControlCommand> command_queue;

  bool res_mpc = base_controller_.run(state_est, reference_trajectory_, controller_params, command_queue);
  computation_success_ = res_mpc;

  if (!res_mpc) {
    ROS_ERROR("[%s] Fail to solve mpc. Switch to PX4_POSITION", pnh_.getNamespace().c_str());
    command_available_ = false;
    command_queue_.clear();
    setControlMode(ControlMode::PX4_POSITION);
    mpc_forbiddened_ = true;
    return;
  }

  command_queue_ = command_queue;
  // command_queue_.pop_front();

  command_available_ = true;
  time_last_command_queue_updated_ = ros::Time::now();


  /**********************************/




}

template <typename Tsystem, typename Tcontroller, typename Tparams>
void Autopilot<Tsystem, Tcontroller, Tparams>::home() {
  const ros::Time time_now = ros::Time::now();
  if (first_time_in_new_flight_mode_) {
    reference_state_ = home_state_;
    time_start_home_ = time_now;    
    first_time_in_new_flight_mode_ = false;
  }

  trackReferenceState(reference_state_);

  if ((time_now - time_start_home_) > ros::Duration(home_timeout_)) {
    setFlightMode(FlightMode::POSITION_TRACK);
  }
}

template <typename Tsystem, typename Tcontroller, typename Tparams>
bool Autopilot<Tsystem, Tcontroller, Tparams>::generateStateWithVirtualFeature(
    vision_common::StateEstimate& state_est,
    Tparams& controller_params) {
  state_est = state_est_;

  Eigen::Vector3d p_B_LB = state_est.orientation.inverse() * (local_goal_ - state_est.position);
  Eigen::Vector3d p_C_LC = q_B_C_.inverse() * (p_B_LB - t_B_C_);
  state_est.gates.gates[0].center = vision_common::VisualFeature(p_C_LC);

  return true;
}

template <typename Tsystem, typename Tcontroller, typename Tparams>
void Autopilot<Tsystem, Tcontroller, Tparams>::land() {
  const ros::Time time_now = ros::Time::now();
  if (first_time_in_new_flight_mode_) {
    reference_state_.position << state_est_.position.x(), state_est_.position.y(), 0.0;
    reference_state_.velocity.setZero();
    reference_state_.heading = vision_common::quaternionToEulerAnglesZYX(
                        state_est_.orientation).z();
    reference_state_.orientation = vision_common::eulerAnglesZYXToQuaternion(
        Eigen::Vector3d(0.0, 0.0, reference_state_.heading)); 
    reference_state_.distance_rate = 0.0; 

    time_start_land_ = time_now;    
    first_time_in_new_flight_mode_ = false;
  }

  trackReferenceState(reference_state_);

  if ((time_now - time_start_land_) > ros::Duration(land_timeout_)) {
    setControlMode(ControlMode::PX4_BODYRATES);
    setAutopilotState(AutopilotState::FLIGHT_TERMINATION);
    time_start_flight_termination_ = time_now;
  }
}

template <typename Tsystem, typename Tcontroller, typename Tparams>
bool Autopilot<Tsystem, Tcontroller, Tparams>::getStateEstimate() {
  if (getFlightMode() == FlightMode::FEATURE_TRACK || getFlightMode() == FlightMode::FEATURE_REACH) {
    return getStateForFeatureTrack();
  } else {
    return getStateForPositionTrack();
  }
}

template <typename Tsystem, typename Tcontroller, typename Tparams>
bool Autopilot<Tsystem, Tcontroller, Tparams>::getStateForPositionTrack() {
  if (!odometry_available_) {
    return false;
  }

  state_est_ = predicted_state_;
  state_est_.collective_thrust = command_.collective_thrust;

  Eigen::Vector3d p_C_LC;
  p_C_LC << 0.0, 0.0, 10.0;
  state_est_.gates.gates.clear();
  state_est_.gates.gates.resize(2);
  state_est_.gates.gates[0].center = vision_common::VisualFeature(p_C_LC);
  state_est_.gates.gates[1].center = vision_common::VisualFeature(p_C_LC);

  odometry_available_ = false;

  return true;
}

template <typename Tsystem, typename Tcontroller, typename Tparams>
bool Autopilot<Tsystem, Tcontroller, Tparams>::getStateForFeatureTrack() {
  if (!odometry_available_ || !point_available_) {
    return false;
  }

  state_est_ = predicted_state_;
  state_est_.collective_thrust = command_.collective_thrust;

  state_est_.gates.gates.clear();
  state_est_.gates.gates.resize(2);
  state_est_.gates.gates[0].center = point_;

  // compute virtual camera point
  Eigen::Quaterniond q_W_B = predicted_state_.orientation;
  Eigen::Vector3d p_C_LC_vir = q_B_C_.inverse() * ( q_W_B * (q_B_C_ * point_.toPoint() + t_B_C_) - t_B_C_);
  vision_common::VisualFeature virtual_point(p_C_LC_vir);

  // std::cout << "p_C_LC: " << point_.toPoint().transpose() << std::endl;

  // std::cout << "p_C_LC_vir: " << p_C_LC_vir.transpose() << std::endl;
  state_est_.gates.gates[1].center = virtual_point;

  odometry_available_ = false;
  point_available_ = false;

  return true;
}


template <typename Tsystem, typename Tcontroller, typename Tparams>
bool Autopilot<Tsystem, Tcontroller, Tparams>::generateState(
    vision_common::StateEstimate& state_est,
    Tparams& controller_params) {
  state_est = state_est_;
  return true;
}

template <typename Tsystem, typename Tcontroller, typename Tparams>
bool Autopilot<Tsystem, Tcontroller, Tparams>::generateReferenceTrajectory(
    const vision_common::TrajectoryPoint& reference_state,
    vision_common::Trajectory& reference_trajectory,
    Tparams& controller_params) {

  controller_params.setReferencePoint(reference_state.position);

  reference_trajectory = vision_common::Trajectory(reference_state);
  reference_trajectory.trajectory_type = vision_common::Trajectory::TrajectoryType::GENERAL;
  
  reference_trajectory_ = reference_trajectory;
  return true;
}

template <typename Tsystem, typename Tcontroller, typename Tparams>
bool Autopilot<Tsystem, Tcontroller, Tparams>::generateReferenceTrajectory(
    const ros::Duration dt,
    vision_common::Trajectory& reference_trajectory,
    Tparams& controller_params) {
  reference_trajectory = vision_common::Trajectory();
  reference_trajectory.trajectory_type = vision_common::Trajectory::TrajectoryType::GENERAL;

  bool lookahead_reached(false);  // Boolean break flag
  double time_wrapover(0.0);
  // Loop over possible trajectories.
  auto it_traj = trajectory_queue_.begin();
  for (; it_traj != trajectory_queue_.end(); it_traj++) {
    // Loop over points on the trajectory
    auto it_points = it_traj->points.begin();
    for (; it_points != it_traj->points.end(); it_points++) {
      // Check wether we reached our lookahead.
      if (it_points->time_from_start.toSec() >
          (dt.toSec() + predictive_control_lookahead_)) {
        lookahead_reached = true;
        break;
      }
      // Add a point if the time corresponds to a sample on the lookahead.
      if (it_points->time_from_start.toSec() >= (dt.toSec() - time_wrapover)) {
        reference_trajectory.points.push_back(*it_points);
        // TODO: track one point at a time
        // break;
      }
    }

    if (lookahead_reached) {
      break;  // Break on boolean flag.
    }
    // Sum up the wrap-ovvr time if lookahead spans multiple trajectories.
    time_wrapover += it_traj->points.back().time_from_start.toSec();
  }

  if (reference_trajectory.points.empty()) {
    ROS_ERROR("reference_trajectory is EMPTY!!!");
    return false;
  }
  // set reference state
  reference_state_ = reference_trajectory.points.front();
  controller_params.setReferencePoint(reference_state_.position);

  reference_trajectory_ = reference_trajectory;

  return true;
}

template <typename Tsystem, typename Tcontroller, typename Tparams>
visualization_msgs::Marker Autopilot<Tsystem, Tcontroller, Tparams>::makeMarker(
    const Eigen::Vector3d& point,
    const Eigen::Vector3d& color) {
  visualization_msgs::Marker msg;
  msg.header.frame_id = "world";
  msg.header.stamp = ros::Time::now();

  msg.type = visualization_msgs::Marker::SPHERE;
  msg.action = visualization_msgs::Marker::ADD;

  msg.pose.orientation.w = 1.0;
  msg.scale.x = 0.3;
  msg.scale.y = 0.3;
  msg.scale.z = 0.3;
  msg.color.a = 1.0;
  msg.color.r = color.x();
  msg.color.g = color.y();
  msg.color.b = color.z();
  msg.id = 0;
  msg.points.clear();

  msg.pose.position.x = point.x();
  msg.pose.position.y = point.y();
  msg.pose.position.z = point.z();  
  msg.pose.orientation.w = 1.0;
  msg.pose.orientation.x = 0.0;
  msg.pose.orientation.y = 0.0;
  msg.pose.orientation.z = 0.0;

  return msg;
}

template <typename Tsystem, typename Tcontroller, typename Tparams>
visualization_msgs::MarkerArray Autopilot<Tsystem, Tcontroller, Tparams>::makeMarkerArray(
    const vision_common::Trajectory& trajectory,
    const Eigen::Vector3d& color) {
  visualization_msgs::MarkerArray array_msg;

  visualization_msgs::Marker msg;
  msg.header.frame_id = "world";
  msg.header.stamp = ros::Time::now();

  msg.type = visualization_msgs::Marker::SPHERE_LIST;
  msg.action = visualization_msgs::Marker::ADD;

  msg.pose.orientation.w = 1.0;
  msg.scale.x = 0.1;
  msg.scale.y = 0.1;
  msg.scale.z = 0.1;
  msg.color.a = 0.8;

  msg.points.clear();
  msg.id = 0;
  msg.color.r = color.x();
  msg.color.g = color.y();
  msg.color.b = color.z();
  auto it_pt = trajectory.points.begin();
  for (; it_pt != trajectory.points.end(); ++it_pt) {
    geometry_msgs::Point pt;
    pt.x = it_pt->position.x();
    pt.y = it_pt->position.y();
    pt.z = it_pt->position.z();
    msg.points.push_back(pt);
  }
  array_msg.markers.push_back(msg);

  return array_msg;
}

template <typename Tsystem, typename Tcontroller, typename Tparams>
void Autopilot<Tsystem, Tcontroller, Tparams>::publishAutopilotFeedback(
    const vision_common::StateEstimate& state_estimate,
    const vision_common::TrajectoryPoint& reference_state,
    vision_common::ControlCommand control_command,
    const ros::Duration& control_command_delay,
    const ros::Duration& control_computation_time,
    const bool success) {

  vision_msgs::AutopilotFeedback fb_msg;
  fb_msg.header.stamp = ros::Time::now();

  switch (autopilot_state_) {
    case AutopilotState::UNINIT:
      fb_msg.autopilot_state = fb_msg.UNINIT;
      break;
    case AutopilotState::BOOT:
      fb_msg.autopilot_state = fb_msg.BOOT;
      break;
    case AutopilotState::CALIBRATING:
      fb_msg.autopilot_state = fb_msg.CALIBRATING;
      break;
    case AutopilotState::STANDBY:
      fb_msg.autopilot_state = fb_msg.STANDBY;
      break;
    case AutopilotState::ACTIVE:
      fb_msg.autopilot_state = fb_msg.ACTIVE;
      break;
    case AutopilotState::CRITICAL:
      fb_msg.autopilot_state = fb_msg.CRITICAL;
      break;
    case AutopilotState::EMERGENCY:
      fb_msg.autopilot_state = fb_msg.EMERGENCY;
      break;
    case AutopilotState::POWEROFF:
      fb_msg.autopilot_state = fb_msg.POWEROFF;
      break;
    case AutopilotState::FLIGHT_TERMINATION:
      fb_msg.autopilot_state = fb_msg.FLIGHT_TERMINATION;
      break;
  }

  switch (control_mode_) {
    case ControlMode::PX4_BODYRATES:
      fb_msg.control_mode = fb_msg.PX4_BODYRATES;
      break;
    case ControlMode::PX4_POSITION:
      fb_msg.control_mode = fb_msg.PX4_POSITION;
      break;
    case ControlMode::FSC_POSITION:
      fb_msg.control_mode = fb_msg.FSC_POSITION;
      break;
  }

  switch (flight_mode_) {
    case FlightMode::POSITION_TRACK:
      fb_msg.flight_mode = fb_msg.POSITION_TRACK;
      break;
    case FlightMode::TRAJECTORY_TRACK:
      fb_msg.flight_mode = fb_msg.TRAJECTORY_TRACK;
      break;
    case FlightMode::VELOCITY_TRACK:
      fb_msg.flight_mode = fb_msg.VELOCITY_TRACK;
      break;
    case FlightMode::FEATURE_TRACK:
      fb_msg.flight_mode = fb_msg.FEATURE_TRACK;
      break;
    case FlightMode::FEATURE_REACH:
      fb_msg.flight_mode = fb_msg.FEATURE_REACH;
      break;
    case FlightMode::HOME:
      fb_msg.flight_mode = fb_msg.HOME;
      break;
    case FlightMode::LAND:
      fb_msg.flight_mode = fb_msg.LAND;
      break;
  }

  fb_msg.state_estimate = state_estimate.toRosMessage();
  fb_msg.reference_state = reference_state.toRosMessage();
  fb_msg.control_command = control_command.toRosMessage();
  fb_msg.control_command_delay = control_command_delay;
  fb_msg.control_computation_time = control_computation_time;
  fb_msg.mpc_success = success;

  autopilot_feedback_pub_.publish(fb_msg);
  time_last_autopilot_feedback_published_ = ros::Time::now();

  vis_drone_pub_.publish(makeMarker(state_estimate.position, Eigen::Vector3d(1,0,0)));
  vis_goal_pub_.publish(makeMarker(reference_state.position, Eigen::Vector3d(0,0,1)));
}

template <typename Tsystem, typename Tcontroller, typename Tparams>
bool Autopilot<Tsystem, Tcontroller, Tparams>::loadParameters() {
  ROS_INFO("-----------Loading FSC_AUTOPILOT Parameters-----------");

#define GET_PARAM(name) \
  if (!vision_common::getParam(#name, name, pnh_)) return false
#define GET_PARAM_(name) \
  if (!vision_common::getParam(#name, name##_, pnh_)) return false

  GET_PARAM_(velocity_estimate_in_world_frame);
  GET_PARAM_(emergency_land_thrust);
  GET_PARAM_(max_emergency_pos_z);
  GET_PARAM_(min_emergency_pos_z);
  GET_PARAM_(max_emergency_pos_xy);
  GET_PARAM_(min_emergency_pos_xy);
  GET_PARAM_(reference_thrust);
  GET_PARAM_(max_reference_pos_z);
  GET_PARAM_(min_reference_pos_z);
  GET_PARAM_(max_reference_pos_xy);
  GET_PARAM_(min_reference_pos_xy);

  GET_PARAM_(max_distance);
  GET_PARAM_(min_distance);
  GET_PARAM_(reference_distance);
  GET_PARAM_(reaching_distance);

  GET_PARAM_(cost_feature_track);
  GET_PARAM_(cost_visibility_u);
  GET_PARAM_(cost_visibility_v);
  GET_PARAM_(cost_constratint_vis);
  GET_PARAM_(cost_constratint_ttc);

  GET_PARAM_(reference_speed);
  GET_PARAM_(cost_speed);
  GET_PARAM_(quad_radius);

  GET_PARAM_(max_thrust_ratio);
  GET_PARAM_(min_thrust_ratio);
  if (max_thrust_ratio_ < min_thrust_ratio_) {
    ROS_ERROR("[%s] max thrust ratio must be larger than min thrust ratio.", pnh_.getNamespace().c_str());
    return false;
  }

  GET_PARAM_(command_publish_frequency);
  GET_PARAM_(watchdog_frequency);
  GET_PARAM_(odometry_estimate_timeout);
  GET_PARAM_(synchron_data_timeout);
  GET_PARAM_(command_update_timeout);
  GET_PARAM_(emergency_landing_timeout);
  GET_PARAM_(flight_termination_timeout);
  GET_PARAM_(land_timeout);
  GET_PARAM_(home_timeout);
  GET_PARAM_(predictive_control_lookahead);
  GET_PARAM_(control_command_delay);

  GET_PARAM_(point_estimate_timeout);
  GET_PARAM_(feature_track_max_distance);
  GET_PARAM_(feature_track_distance_threshold);
  GET_PARAM_(global_point_position_change_threshold);

  GET_PARAM_(image_u_bound);
  GET_PARAM_(image_v_bound);

  GET_PARAM_(max_planning_vel);
  GET_PARAM_(use_flip_mode);
  GET_PARAM_(height_gap);

  double init_heading;
  GET_PARAM(init_heading);
  reference_state_.heading = init_heading;
  reference_state_.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(
      vision_common::wrapMinusPiToPi(init_heading), Eigen::Vector3d::UnitZ()));

  int control_mode;
  GET_PARAM(control_mode);
  switch (control_mode) {
    case 0:
      setControlMode(ControlMode::PX4_BODYRATES);
      break;
    case 1:
      setControlMode(ControlMode::PX4_POSITION);
      break;
    case 2:
      setControlMode(ControlMode::FSC_POSITION);
      break;
    default:
      ROS_ERROR("[%s] Unknown control mode.", pnh_.getNamespace().c_str());
      return false;
      break;
  }

  std::vector<double> t_B_C(3), q_B_C(4);
  if (!pnh_.getParam("t_B_C", t_B_C)) {
    ROS_WARN("Camera extrinsic translation is not set.");
    return false;
  } else {
    t_B_C_ << t_B_C[0], t_B_C[1], t_B_C[2];
  }
  if (!pnh_.getParam("q_B_C", q_B_C)) {
    ROS_WARN("Camera extrinsic rotation is not set.");
    return false;
  } else {
    q_B_C_ = Eigen::Quaterniond(q_B_C[0], q_B_C[1], q_B_C[2], q_B_C[3]);
  }

  // add initial goal position and landmarks
  std::vector<double> pos(3);
  if (!pnh_.getParam("init_position", pos)) {
    ROS_WARN("Initial position is not set.");
    return false;
  } else {
    reference_state_.position << pos[0], pos[1], pos[2];
  }

  double home_heading;
  GET_PARAM(home_heading);
  if (!pnh_.getParam("home_position", pos)) {
    ROS_WARN("Autopilot: home position is not set.");
  } else {
    home_state_.position = Eigen::Vector3d(pos[0], pos[1], pos[2]);
    home_state_.velocity.setZero();
    home_state_.heading = home_heading;
    home_state_.orientation = vision_common::eulerAnglesZYXToQuaternion(
      Eigen::Vector3d(0.0, 0.0, home_heading));
    home_state_.distance_rate = 0.0; 
  }

  vision_common::getParam("print_info", print_info_, false, pnh_);
  if (print_info_) {
    ROS_INFO("Informative printing enabled.");
  }

  vision_common::getParam("enable_rc_tune", enable_rc_tune_, false, pnh_);
  if (enable_rc_tune_) {
    ROS_INFO("Enable RC tune.");
  }

  vision_common::getParam("enable_state_prediction", enable_state_prediction_, false, pnh_);
  if (enable_state_prediction_) {
    ROS_INFO("Enable state prediction.");
  }

  vision_common::getParam("enable_trajectory_replan", enable_trajectory_replan_, false, pnh_);
  if (enable_trajectory_replan_) {
    ROS_INFO("Enable trajectory replan.");
  }

  vision_common::getParam("enable_moving_target_track", enable_moving_target_track_, false, pnh_);
  if (enable_moving_target_track_) {
    ROS_INFO("Enable moving target tracking.");
  }

  if (!base_controller_params_.loadParameters(pnh_)) {
    return false;
  }

  // TODO: set control params for feature tracking
  feature_track_params_ = base_controller_params_;  
  feature_track_params_.setPositionWeight(0.0, 0.0);
  feature_track_params_.setVisualServoWeight(cost_feature_track_);
  feature_track_params_.setPerceptionWeight(cost_visibility_u_, cost_visibility_v_);
  feature_track_params_.setSlackVariableWeight(cost_constratint_vis_, cost_constratint_ttc_);
  feature_track_params_.setReferenceSpeed(0.0);
  feature_track_params_.setVelocityWeight(10.0);

  mpcc_params_ = base_controller_params_;
  mpcc_params_.setPositionWeight(0.0, 0.0);
  mpcc_params_.setVisualServoWeight(cost_feature_track_);
  mpcc_params_.setPerceptionWeight(cost_visibility_u_, cost_visibility_v_);
  // mpcc_params_.setSlackVariableWeight(cost_constratint_vis_, cost_constratint_ttc_);

  mpcc_params_.setSv1Weight(cost_constratint_vis_);
  mpcc_params_.setSv2Weight(cost_constratint_ttc_);

  mpcc_params_.setReferenceSpeed(cost_speed_);
  mpcc_params_.setVelocityWeight(1.0);

  Eigen::Vector3d euler(-M_PI, 0, -M_PI/2);
  Eigen::Quaterniond quat = vision_common::eulerAnglesZYXToQuaternion(euler);
  std::cout << "quat: " << quat.w() << ", " << quat.x() << ", " << quat.y() << ", " << quat.z() << std::endl;

#undef GET_PARAM
#undef GET_PARAM_

  return true;
}

template <typename Tsystem, typename Tcontroller, typename Tparams>
double Autopilot<Tsystem, Tcontroller, Tparams>::computeGaussianScale(const double val, const double mean, const double std) {
  double val1 = 1.0 / (std::sqrt(2*M_PI) * std);
  double val2 = -(val - mean)*(val - mean)/(2*std*std);
  return val1 * std::exp(val2);
}

template <typename Tsystem, typename Tcontroller, typename Tparams>
double Autopilot<Tsystem, Tcontroller, Tparams>::computeDistanceToPlane(
    const Eigen::Vector3d& pt,
    const Eigen::Vector3d& ppos,
    const Eigen::Quaterniond& porient) {

  Eigen::Vector3d pt_g = porient.inverse() * (pt - ppos);
  return pt_g.z();
  // return (ppos - pt).norm();
}

template <typename Tsystem, typename Tcontroller, typename Tparams>
bool Autopilot<Tsystem, Tcontroller, Tparams>::selectGate(
    const vision_common::GateFeatureArray& gates,
    const Eigen::Vector3d& position,
    vision_common::GateFeature& gate_selected) {
  

  bool found = false;
  for (int i = current_gate_index_; i < (int)gates.gates.size(); ++i) {
    double distance_to_gate = computeDistanceToPlane(position, gates.gates[i].translation, gates.gates[i].rotation);
    // the first gate that has postivie distance
    if (distance_to_gate > 0.0) {
      gate_selected = gates.gates[i];
      if (i > current_gate_index_) {
        current_gate_index_ = i;
      }
      // std::cout << "gate " << i << " selected" << std::endl;
      found = true;
      break;
    }  
  }

  // for (const auto& gate : gates.gates) {
  //   double distance_to_gate = computeDistanceToPlane(position, gate.translation, gate.rotation);
  //   // the first gate that has postivie distance
  //   if (distance_to_gate > 0.0) {
  //     gate_selected = gate;
  //     found = true;
  //     break;
  //   }
  // }

  if (!found) {
    return false;
  }

  // gate_selected = gates.gates.front();
  return true;
}


}  // namespace fsc

