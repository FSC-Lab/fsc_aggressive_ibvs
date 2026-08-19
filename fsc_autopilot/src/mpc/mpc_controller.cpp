#include <mpc/mpc_controller.h>

#include <ctime>

namespace mpc {

// T is the variable type
template <typename T>
MpcController<T>::MpcController(const ros::NodeHandle& nh,
                                const ros::NodeHandle& pnh,
                                const std::string& topic)
    : nh_(nh),
      pnh_(pnh),
      mpc_wrapper_(MpcWrapper<T>()),
      timing_feedback_(T(1e-3)),
      timing_preparation_(T(1e-3)),  //TODO: MPC 
      est_state_((Eigen::Matrix<T, kStateSize, 1>()
                    << 0, 0, 0,          // position
                       0,                // psi (heading)
                       0, 0, 0,          // velocity
                       1.0, 0.0, 0.0, 0.0, // bearing quaternion
                       10.0,             // range
                       0.0,              // gimbal angle
                       0.0, 0.0).finished()), // slack integrators
      reference_states_(Eigen::Matrix<T, kCostSize, kSamples + 1>::Zero()),
      reference_inputs_(Eigen::Matrix<T, kInputSize, kSamples + 1>::Zero()),
      predicted_states_(Eigen::Matrix<T, kStateSize, kSamples + 1>::Zero()),
      predicted_inputs_(Eigen::Matrix<T, kInputSize, kSamples>::Zero()) {
  // pub_predicted_trajectory_ = nh_.advertise<nav_msgs::Path>(topic, 1);

  // // Turn off the autopilot
  // sub_autopilot_off_ = nh_.subscribe("autopilot/off", 1,
  //                                    &MpcController<T>::offCallback, this);

  if (!params_.loadParameters(pnh_)) {
    ROS_ERROR("[%s] Could not load MPC_CONTROLLER parameters.", pnh_.getNamespace().c_str());
    ros::shutdown();
    return;
  }

  setNewParams(params_);

  solve_from_scratch_ = true;
  preparation_thread_ = std::thread(&MpcWrapper<T>::prepare, mpc_wrapper_);

  std::cout << "Q_: \n" << params_.Q_ << std::endl;
  std::cout << "R_: \n" << params_.R_ << std::endl;
}

template <typename T>
MpcController<T>::~MpcController() {
  if (preparation_thread_.joinable()) preparation_thread_.join();
}


template <typename T>
void MpcController<T>::reset() {
  solve_from_scratch_ = true;
}

// template <typename T>
// void MpcController<T>::offCallback(const std_msgs::Empty::ConstPtr& msg) {
//   solve_from_scratch_ = true;
// }

template <typename T>
vision_common::ControlCommand MpcController<T>::off() {
  vision_common::ControlCommand command;

  command.zero();

  return command;
}

template <typename T>
bool MpcController<T>::run(
    const vision_common::StateEstimate& state_est,
    const vision_common::Trajectory& trajectory,
    MpcParams<T>& params,
    std::list<vision_common::ControlCommand>& command_queue,
    std::vector<vision_common::StateEstimate>& predicted_states) {
  ros::Time call_time = ros::Time::now();
  const clock_t start = clock();
  if (params.changed_) {
    params_ = params;
    setNewParams(params);
  }

  preparation_thread_.join();
  setState(state_est);
  setReference(trajectory);

  static const bool do_preparation_step(false);
  // Get the feedback from MPC.
  mpc_wrapper_.setTrajectory(reference_states_, reference_inputs_);
  if (solve_from_scratch_) {
    ROS_INFO("Solving VPC with hover as initial guess.");
    bool res = mpc_wrapper_.solve(est_state_);
    if (!res) {
      ROS_WARN("Fail to solve VPC due to solver.");
      return false;
    }
    solve_from_scratch_ = false;
  } else {
    bool res = mpc_wrapper_.update(est_state_, do_preparation_step);
    if (!res) {
      ROS_WARN("Fail to update VPC due to solver.");
      return false;
    }
  }

  mpc_wrapper_.getStates(predicted_states_);
  mpc_wrapper_.getInputs(predicted_inputs_);

  // Warm-start the gimbal-angle feedback from the model's own prediction
  // (used next tick when no encoder feedback is pushed via setGimbalAngle).
  gimbal_angle_ = predicted_states_((int)StateIndex::GIMBAL, 1);

  // Start a thread to prepare for the next execution.
  preparation_thread_ =
      std::thread(&MpcController<T>::preparationThread, this);

  command_queue.clear();
  predicted_states.clear();

  T dt = mpc_wrapper_.getTimestep();
  vision_common::ControlCommand command;
  vision_common::StateEstimate state;

  bool no_nan_control = true;
  for (int i = 0; i < kSamples; ++i, call_time += ros::Duration(dt)) {
    no_nan_control = getControlCommand(predicted_states_.col(i),
                                predicted_inputs_.col(i),
                                call_time, command);
    if (!no_nan_control) {
      break;
    }                 
    command_queue.push_back(command);
  }

  bool no_nan_state = true;
  for (int i = 0; i < kSamples+1; ++i, call_time += ros::Duration(dt)) {
    no_nan_state = getPredictedState(predicted_states_.col(i), call_time, state);
    if (!no_nan_state) {
      break;
    }  
    predicted_states.push_back(state);
  }

  // Timing
  const clock_t end = clock();
  timing_feedback_ =
      0.9 * timing_feedback_ + 0.1 * double(end - start) / CLOCKS_PER_SEC;
  if (params_.print_info_) {
    ROS_INFO_THROTTLE(1.0,
                      "NMPC Timing: Latency: %1.1f ms  |  Total: %1.1f ms",
                      timing_feedback_ * 1000,
                      (timing_feedback_ + timing_preparation_) * 1000);
  }

  if (!no_nan_control || !no_nan_state) {
    ROS_ERROR("NaN exist in the MPC solution.");
    return false;
  }

  return true;
}

template <typename T>
bool MpcController<T>::run(
    const vision_common::StateEstimate& state_est,
    const vision_common::Trajectory& trajectory,
    MpcParams<T>& params,
    std::list<vision_common::ControlCommand>& command_queue) {
  ros::Time call_time = ros::Time::now();
  const clock_t start = clock();
  if (params.changed_) {
    params_ = params;
    setNewParams(params);
  }

  preparation_thread_.join();
  setState(state_est);
  setReference(trajectory);

  static const bool do_preparation_step(false);
  // Get the feedback from MPC.
  mpc_wrapper_.setTrajectory(reference_states_, reference_inputs_);
  if (solve_from_scratch_) {
    ROS_INFO("Solving VPC with hover as initial guess.");
    bool res = mpc_wrapper_.solve(est_state_);
    if (!res) {
      ROS_WARN("Fail to solve VPC due to solver.");
      return false;
    }
    solve_from_scratch_ = false;
  } else {
    bool res = mpc_wrapper_.update(est_state_, do_preparation_step);
    if (!res) {
      ROS_WARN("Fail to update VPC due to solver.");
      return false;
    }
  }

  mpc_wrapper_.getStates(predicted_states_);
  mpc_wrapper_.getInputs(predicted_inputs_);

  // Warm-start the gimbal-angle feedback from the model's own prediction
  // (used next tick when no encoder feedback is pushed via setGimbalAngle).
  gimbal_angle_ = predicted_states_((int)StateIndex::GIMBAL, 1);

  // Start a thread to prepare for the next execution.
  preparation_thread_ =
      std::thread(&MpcController<T>::preparationThread, this);

  command_queue.clear();

  T dt = mpc_wrapper_.getTimestep();
  vision_common::ControlCommand command;
  vision_common::StateEstimate state;

  bool no_nan_control = true;
  for (int i = 0; i < kSamples; ++i, call_time += ros::Duration(dt)) {
    no_nan_control = getControlCommand(predicted_states_.col(i),
                                predicted_inputs_.col(i),
                                call_time, command);
    if (!no_nan_control) {
      break;
    }                 
    command_queue.push_back(command);
  }

  bool no_nan_state = true;
  for (int i = 0; i < kSamples+1; ++i, call_time += ros::Duration(dt)) {
    no_nan_state = getPredictedState(predicted_states_.col(i), call_time, state);
    if (!no_nan_state) {
      break;
    } 
  }

  // Timing
  const clock_t end = clock();
  timing_feedback_ =
      0.9 * timing_feedback_ + 0.1 * double(end - start) / CLOCKS_PER_SEC;
  if (params_.print_info_) {
    ROS_INFO_THROTTLE(1.0,
                      "NMPC Timing: Latency: %1.1f ms  |  Total: %1.1f ms",
                      timing_feedback_ * 1000,
                      (timing_feedback_ + timing_preparation_) * 1000);
  }

  if (!no_nan_control || !no_nan_state) {
    ROS_ERROR("NaN exist in the MPC solution.");
    return false;
  }

  return true;
}

template <typename T>
bool MpcController<T>::setState(
    const vision_common::StateEstimate& state_est) {

  const vision_common::GateFeatureArray& gates = state_est.gates;
  est_state_((int)StateIndex::POS_X) = state_est.position.x();
  est_state_((int)StateIndex::POS_Y) = state_est.position.y();
  est_state_((int)StateIndex::POS_Z) = state_est.position.z();
  // Body heading (yaw) extracted from the odometry attitude quaternion.
  est_state_((int)StateIndex::PSI) =
      vision_common::quaternionToEulerAnglesZYX(state_est.orientation).z();
  est_state_((int)StateIndex::VEL_X) = state_est.velocity.x();
  est_state_((int)StateIndex::VEL_Y) = state_est.velocity.y();
  est_state_((int)StateIndex::VEL_Z) = state_est.velocity.z();
  // Bearing (line-of-sight) quaternion + range measured from the camera.
  est_state_((int)StateIndex::BEA_W) = gates.gates[0].center.normal_vector.orientation.w();
  est_state_((int)StateIndex::BEA_X) = gates.gates[0].center.normal_vector.orientation.x();
  est_state_((int)StateIndex::BEA_Y) = gates.gates[0].center.normal_vector.orientation.y();
  est_state_((int)StateIndex::BEA_Z) = gates.gates[0].center.normal_vector.orientation.z();
  est_state_((int)StateIndex::DISTANCE) = gates.gates[0].center.distance;
  // Gimbal pitch feedback (encoder if available, else last prediction).
  est_state_((int)StateIndex::GIMBAL) = gimbal_angle_;

  est_state_((int)StateIndex::SLA_1) = 0.0;
  est_state_((int)StateIndex::SLA_2) = 0.0;

  const bool bearing_norm_ok =
      abs(est_state_.segment((int)StateIndex::BEA_W, 4).norm() - 1.0) < 0.1;

  return bearing_norm_ok;
}

template <typename T>
bool MpcController<T>::setReference(
    const vision_common::Trajectory& reference_trajectory) {
  reference_states_.setZero();
  reference_inputs_.setZero();

  const T dt = mpc_wrapper_.getTimestep();

  // Cost-state reference layout (14):
  //   p(3), psi, v(3), img_u, img_v, err(3), rdot1, theta_g
  // img_uv and err targets are zero (center the target, zero visual error);
  // the gimbal is regularized toward level (0).
  auto make_ref = [](const vision_common::TrajectoryPoint& pt) {
    const T yaw = (T)vision_common::quaternionToEulerAnglesZYX(pt.orientation).z();
    return (Eigen::Matrix<T, kCostSize, 1>() <<
              pt.position.template cast<T>(),
              yaw,
              pt.velocity.template cast<T>(),
              (T)0.0, (T)0.0,                // img_uv
              (T)0.0, (T)0.0, (T)0.0,        // visual err
              (T)pt.distance_rate,           // desired closing rate
              (T)0.0).finished();            // gimbal angle ref
  };

  if (reference_trajectory.points.size() == 1) {
    const vision_common::TrajectoryPoint reference_state(
        reference_trajectory.points.front());

    reference_states_ = make_ref(reference_state).replicate(1, kSamples + 1);

    // Feedforward commanded velocity; zero yaw-rate / gimbal-rate / slacks.
    reference_inputs_ = (Eigen::Matrix<T, kInputSize, 1>()
        << reference_state.velocity.template cast<T>(),
           (T)0.0, (T)0.0, (T)0.0, (T)0.0).finished().replicate(1, kSamples + 1);
  } else {
    auto iterator(reference_trajectory.points.begin());
    ros::Duration t_start = reference_trajectory.points.begin()->time_from_start;
    auto last_element = std::prev(reference_trajectory.points.end());
    for (int i = 0; i < kSamples + 1; i++) {
      while ((iterator->time_from_start - t_start).toSec() <= i * dt &&
             iterator != last_element) {
        iterator++;
      }

      reference_states_.col(i) = make_ref(*iterator);
      reference_inputs_.col(i) << iterator->velocity.template cast<T>(),
                                  (T)0.0, (T)0.0, (T)0.0, (T)0.0;
    }
  }

  return true;
}

template <typename T>
bool MpcController<T>::getControlCommand(
    const Eigen::Ref<const Eigen::Matrix<T, kStateSize, 1>> state,
    const Eigen::Ref<const Eigen::Matrix<T, kInputSize, 1>> input,
    ros::Time& time,
    vision_common::ControlCommand& command) {
  Eigen::Matrix<T, kInputSize, 1> input_bounded = input.template cast<T>();

  // Commanded world velocity, yaw rate, and gimbal-rate integrated into a
  // gimbal-angle setpoint.
  T vel_x = input_bounded((int)ControlIndex::VEL_X);
  T vel_y = input_bounded((int)ControlIndex::VEL_Y);
  T vel_z = input_bounded((int)ControlIndex::VEL_Z);
  T yaw_rate = input_bounded((int)ControlIndex::YAW_RATE);
  T gimbal_rate = input_bounded((int)ControlIndex::GIMBAL_RATE);
  T gimbal_angle = state((int)StateIndex::GIMBAL);

  if (std::isnan(vel_x) || std::isnan(vel_y) || std::isnan(vel_z) ||
      std::isnan(yaw_rate) || std::isnan(gimbal_rate) ||
      std::isnan(gimbal_angle)) {
    return false;
  }

  // Bound inputs for sanity.
  vel_x = std::max(-params_.max_velocity_, std::min(params_.max_velocity_, vel_x));
  vel_y = std::max(-params_.max_velocity_, std::min(params_.max_velocity_, vel_y));
  vel_z = std::max(-params_.max_velocity_, std::min(params_.max_velocity_, vel_z));
  yaw_rate = std::max(-params_.max_yaw_rate_, std::min(params_.max_yaw_rate_, yaw_rate));
  gimbal_angle = std::max(params_.gimbal_angle_min_,
                          std::min(params_.gimbal_angle_max_, gimbal_angle));

  command = vision_common::ControlCommand();
  command.timestamp = time;
  command.armed = true;
  command.control_mode = vision_common::ControlMode::VELOCITY;
  command.expected_execution_time = time;
  command.velocity.x() = vel_x;
  command.velocity.y() = vel_y;
  command.velocity.z() = vel_z;
  command.yaw_rate = yaw_rate;
  command.gimbal_angle = gimbal_angle;
  command.slack_variables.resize(2);
  command.slack_variables[0] = input_bounded((int)ControlIndex::SLA_1);
  command.slack_variables[1] = input_bounded((int)ControlIndex::SLA_2);

  return true;
}

template <typename T>
bool MpcController<T>::getPredictedState(
    const Eigen::Ref<const Eigen::Matrix<T, kStateSize, 1>> state,
    ros::Time& time,
    vision_common::StateEstimate& predicted_state) {

  T pos_x = state((int)StateIndex::POS_X);
  T pos_y = state((int)StateIndex::POS_Y);
  T pos_z = state((int)StateIndex::POS_Z);
  T psi = state((int)StateIndex::PSI);
  T vel_x = state((int)StateIndex::VEL_X);
  T vel_y = state((int)StateIndex::VEL_Y);
  T vel_z = state((int)StateIndex::VEL_Z);
  T distance = state((int)StateIndex::DISTANCE);

  if (std::isnan(pos_x) ||
      std::isnan(pos_y) ||
      std::isnan(pos_z) ||
      std::isnan(psi) ||
      std::isnan(vel_x) ||
      std::isnan(vel_y) ||
      std::isnan(vel_z) ||
      std::isnan(distance)) {
    return false;
  }

  predicted_state = vision_common::StateEstimate();
  predicted_state.timestamp = time;
  predicted_state.position.x() = pos_x;
  predicted_state.position.y() = pos_y;
  predicted_state.position.z() = pos_z;
  // Level body: orientation is a pure yaw about world z.
  predicted_state.orientation =
      Eigen::Quaternion<double>(Eigen::AngleAxis<double>(
          (double)psi, Eigen::Vector3d::UnitZ()));
  predicted_state.velocity.x() = vel_x;
  predicted_state.velocity.y() = vel_y;
  predicted_state.velocity.z() = vel_z;

  predicted_state.gates.gates.resize(1);
  predicted_state.gates.gates[0].center.normal_vector.orientation.w() = state((int)StateIndex::BEA_W);
  predicted_state.gates.gates[0].center.normal_vector.orientation.x() = state((int)StateIndex::BEA_X);
  predicted_state.gates.gates[0].center.normal_vector.orientation.y() = state((int)StateIndex::BEA_Y);
  predicted_state.gates.gates[0].center.normal_vector.orientation.z() = state((int)StateIndex::BEA_Z);
  predicted_state.gates.gates[0].center.distance = distance;

  return true;
}


template <typename T>
void MpcController<T>::preparationThread() {
  const clock_t start = clock();

  mpc_wrapper_.prepare();

  // Timing
  const clock_t end = clock();
  timing_preparation_ =
      0.9 * timing_preparation_ + 0.1 * double(end - start) / CLOCKS_PER_SEC;
}

template <typename T>
bool MpcController<T>::setNewParams(MpcParams<T>& params) {

  mpc_wrapper_.setSv2ScaledCosts(params.Q_, params.R_, params.sv2_scalings_);

  mpc_wrapper_.setLimits(params.max_velocity_,
                         params.max_yaw_rate_,
                         params.max_gimbal_rate_);
  mpc_wrapper_.setCameraParameters(params.t_B_C_, params.q_B_M_);
  mpc_wrapper_.setFocusParameters(params.focus_p_);
  for (int i = 0; i < kSamples+1; ++i) {
    mpc_wrapper_.setTrajectoryParameters(i, params.polynomial_trajectory_[i].poly_x,
                                            params.polynomial_trajectory_[i].poly_y,
                                            params.polynomial_trajectory_[i].poly_z,
                                            params.polynomial_trajectory_[i].distance_start);
  }

  params.changed_ = false;

  if (params_.print_info_) {
    std::cout << "Q_: \n" << params_.Q_ << std::endl;
    std::cout << "R_: \n" << params_.R_ << std::endl;
  }

  return true;
}

template class MpcController<float>;
template class MpcController<double>;

}  // namespace mpc