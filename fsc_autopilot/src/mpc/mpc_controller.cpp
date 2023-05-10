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
                    << 0, 0, 0,
                       1, 0, 0, 0,
                       0, 0, 0,
                       9.81,
                       1.0, 0.0, 0.0, 0.0, // bearing vector
                       10.0,
                       0.0, 0.0).finished()),
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

//TODO: MPC 
template <typename T>
bool MpcController<T>::setState(
    const vision_common::StateEstimate& state_est) {

  const vision_common::GateFeatureArray& gates = state_est.gates;
  est_state_((int)StateIndex::POS_X) = state_est.position.x();
  est_state_((int)StateIndex::POS_Y) = state_est.position.y();
  est_state_((int)StateIndex::POS_Z) = state_est.position.z();
  est_state_((int)StateIndex::ROT_W) = state_est.orientation.w();
  est_state_((int)StateIndex::ROT_X) = state_est.orientation.x();
  est_state_((int)StateIndex::ROT_Y) = state_est.orientation.y();
  est_state_((int)StateIndex::ROT_Z) = state_est.orientation.z();
  est_state_((int)StateIndex::VEL_X) = state_est.velocity.x();
  est_state_((int)StateIndex::VEL_Y) = state_est.velocity.y();
  est_state_((int)StateIndex::VEL_Z) = state_est.velocity.z();
  est_state_((int)StateIndex::THRUST) = state_est.collective_thrust;
  // actual feature
  est_state_((int)StateIndex::FOC_W) = gates.gates[0].center.normal_vector.orientation.w();
  est_state_((int)StateIndex::FOC_X) = gates.gates[0].center.normal_vector.orientation.x();
  est_state_((int)StateIndex::FOC_Y) = gates.gates[0].center.normal_vector.orientation.y();
  est_state_((int)StateIndex::FOC_Z) = gates.gates[0].center.normal_vector.orientation.z();
  est_state_((int)StateIndex::DISTANCE) = gates.gates[0].center.distance;
  // virtual feature
  // est_state_((int)StateIndex::FOC_W) = gates.gates[1].center.normal_vector.orientation.w();
  // est_state_((int)StateIndex::FOC_X) = gates.gates[1].center.normal_vector.orientation.x();
  // est_state_((int)StateIndex::FOC_Y) = gates.gates[1].center.normal_vector.orientation.y();
  // est_state_((int)StateIndex::FOC_Z) = gates.gates[1].center.normal_vector.orientation.z();
  // est_state_((int)StateIndex::DISTANCE) = gates.gates[1].center.distance;

  est_state_((int)StateIndex::SLA_1) = 0.0;
  est_state_((int)StateIndex::SLA_2) = 0.0;
  
  const bool quaternion_norm_ok =
      abs(est_state_.segment((int)StateIndex::ROT_W, 4).norm() - 1.0) < 0.1;

  return quaternion_norm_ok;
}

//TODO: MPC 
template <typename T>
bool MpcController<T>::setReference(
    const vision_common::Trajectory& reference_trajectory) {
  reference_states_.setZero();
  reference_inputs_.setZero();

  const T dt = mpc_wrapper_.getTimestep();
  Eigen::Matrix<T, 3, 1> acceleration;
  const Eigen::Matrix<T, 3, 1> gravity(0.0, 0.0, -9.81);
  Eigen::Quaternion<T> q_orientation;
  T reference_thrust;

  bool quaternion_norm_ok(true);

  if (reference_trajectory.points.size() == 1) {
    // Take the first point as the reference point
    vision_common::TrajectoryPoint reference_state(reference_trajectory.points.front());

    // R_wb = Rz*Ry*Rx
    q_orientation = reference_state.orientation.template cast<T>();
    // acceleration << reference_state.acceleration.template cast<T>() - gravity;
    // reference_thrust = acceleration.norm();

    reference_states_ =
        (Eigen::Matrix<T, kCostSize, 1>() <<
          reference_state.position.template cast<T>(),
          // 0.0, 0.0, 0.0,
          q_orientation.w(),
          q_orientation.x(),
          q_orientation.y(),
          q_orientation.z(),
          reference_state.velocity.template cast<T>(),
          est_state_((int)StateIndex::THRUST),
          0.0, 0.0,
          reference_state.position.template cast<T>(),
          // 0.0, 0.0, 0.0,
          reference_state.distance_rate).finished().replicate(1, kSamples + 1);


    // std::cout << "-----------setReference-----------" << std::endl;
    // std::cout.precision(3);
    // std::cout << "Thrust Ref.: " << reference_thrust << std::endl;

    for (int i = 0; i < kSamples + 1; i++) {
      if (reference_states_.col(i).segment((int)ReferenceIndex::ROT_W, 4).dot(est_state_.segment((int)StateIndex::ROT_W, 4)) < 0.0) {
        reference_states_.block((int)ReferenceIndex::ROT_W, i, 4, 1) = -reference_states_.block((int)ReferenceIndex::ROT_W, i, 4, 1);
      }
    }

    quaternion_norm_ok = abs(q_orientation.norm() - 1.0) < 0.1;

    double ref_sv = 0.0;
    reference_inputs_ = (Eigen::Matrix<T, kInputSize, 1>()
                            << 0.0, reference_state.bodyrates.template cast<T>(), ref_sv, ref_sv).finished().replicate(1, kSamples + 1);
  } else {
    auto iterator(reference_trajectory.points.begin());
    ros::Duration t_start = reference_trajectory.points.begin()->time_from_start;
    auto last_element = reference_trajectory.points.end();
    last_element = std::prev(last_element);
    auto max_ref_speed = 0.0;
    for (int i = 0; i < kSamples + 1; i++) {
      while ((iterator->time_from_start - t_start).toSec() <= i * dt && iterator != last_element) {
        iterator++;
      }

      q_orientation = iterator->orientation.template cast<T>();

      reference_states_.col(i) << iterator->position.template cast<T>(),
                                  // 0.0, 0.0, 0.0,
                                  q_orientation.w(),
                                  q_orientation.x(),
                                  q_orientation.y(),
                                  q_orientation.z(),
                                  iterator->velocity.template cast<T>(),
                                  est_state_((int)StateIndex::THRUST),
                                  0.0, 0.0,
                                  iterator->position.template cast<T>(),
                                  // 0.0, 0.0, 0.0,
                                  iterator->distance_rate;

      if (reference_states_.col(i).segment((int)ReferenceIndex::ROT_W, 4).dot(est_state_.segment((int)StateIndex::ROT_W, 4)) < 0.0) {
        reference_states_.block((int)ReferenceIndex::ROT_W, i, 4, 1) = -reference_states_.block((int)ReferenceIndex::ROT_W, i, 4, 1);
      }
      if (max_ref_speed < iterator->velocity.norm()) {
        max_ref_speed = iterator->velocity.norm();
      }

      double ref_sv = 0.0;
      reference_inputs_.col(i) << 0.0, iterator->bodyrates.template cast<T>(), ref_sv, ref_sv;
      quaternion_norm_ok &= abs(q_orientation.norm() - 1.0) < 0.1;
    }

    // std::cout << "max_ref_speed: " << max_ref_speed << std::endl;
  }

  return quaternion_norm_ok;
}

//TODO: MPC 
template <typename T>
bool MpcController<T>::getControlCommand(
    const Eigen::Ref<const Eigen::Matrix<T, kStateSize, 1>> state,
    const Eigen::Ref<const Eigen::Matrix<T, kInputSize, 1>> input,
    ros::Time& time,
    vision_common::ControlCommand& command) {
  Eigen::Matrix<T, kInputSize, 1> input_bounded = input.template cast<T>();
  const T dt = mpc_wrapper_.getTimestep();

  T rot_w = state((int)StateIndex::ROT_W);
  T rot_x = state((int)StateIndex::ROT_X);
  T rot_y = state((int)StateIndex::ROT_Y);
  T rot_z = state((int)StateIndex::ROT_Z);
  T thrust = state((int)StateIndex::THRUST);
  T thrust_rate = input_bounded((int)ControlIndex::THRUST_DOT);
  T bodyrate_x = input_bounded((int)ControlIndex::RAT_X);
  T bodyrate_y = input_bounded((int)ControlIndex::RAT_Y);
  T bodyrate_z = input_bounded((int)ControlIndex::RAT_Z);

  // std::cout << "-----------RawCommand-----------" << std::endl;
  // std::cout.precision(3);
  // std::cout << "Thrust: " << thrust << std::endl;
  // std::cout << "Thrust Rate: " << thrust_rate << std::endl;

  if (std::isnan(rot_w) ||
      std::isnan(rot_x) ||
      std::isnan(rot_y) ||
      std::isnan(rot_z) ||
      std::isnan(thrust) ||
      std::isnan(thrust_rate) ||
      std::isnan(bodyrate_x) ||
      std::isnan(bodyrate_y) ||
      std::isnan(bodyrate_z)) {
    return false;
  }

  // Bound inputs for sanity.
  thrust_rate = std::max(-params_.max_thrust_rate_, std::min(params_.max_thrust_rate_, thrust_rate));
  bodyrate_x = std::max(-params_.max_bodyrate_x_, std::min(params_.max_bodyrate_x_, bodyrate_x));
  bodyrate_y = std::max(-params_.max_bodyrate_y_, std::min(params_.max_bodyrate_y_, bodyrate_y));
  bodyrate_z = std::max(-params_.max_bodyrate_z_, std::min(params_.max_bodyrate_z_, bodyrate_z));

  // thrust_rate = std::max(T(-1), std::min(T(1), thrust_rate));
  thrust = std::max(params_.min_thrust_, std::min(params_.max_thrust_, thrust + dt * thrust_rate));
  // thrust = std::max(params_.min_thrust_, std::min(params_.max_thrust_, thrust));

  command = vision_common::ControlCommand();
  command.timestamp = time;
  command.armed = true;
  command.control_mode = vision_common::ControlMode::BODY_RATES;
  command.expected_execution_time = time;
  command.collective_thrust = thrust;
  command.thrust_rate = thrust_rate;
  command.bodyrates.x() = bodyrate_x;
  command.bodyrates.y() = bodyrate_y;
  command.bodyrates.z() = bodyrate_z;
  command.orientation.w() = rot_w;
  command.orientation.x() = rot_x;
  command.orientation.y() = rot_y;
  command.orientation.z() = rot_z;
  command.slack_variables.resize(2);
  command.slack_variables[0] = input_bounded((int)ControlIndex::SLA_1);
  command.slack_variables[1] = input_bounded((int)ControlIndex::SLA_2);

  // std::cout << "sv1: " << command.slack_variables[0] << std::endl;
  // if (command.slack_variables[1] > 1e-2)
  //   std::cout << "sv2: " << command.slack_variables[1] << std::endl;

  return true;
}

//TODO: MPC 
template <typename T>
bool MpcController<T>::getPredictedState(
    const Eigen::Ref<const Eigen::Matrix<T, kStateSize, 1>> state,
    ros::Time& time,
    vision_common::StateEstimate& predicted_state) {

  T pos_x = state((int)StateIndex::POS_X);
  T pos_y = state((int)StateIndex::POS_Y);
  T pos_z = state((int)StateIndex::POS_Z);
  T rot_w = state((int)StateIndex::ROT_W);
  T rot_x = state((int)StateIndex::ROT_X);
  T rot_y = state((int)StateIndex::ROT_Y);
  T rot_z = state((int)StateIndex::ROT_Z);
  T vel_x = state((int)StateIndex::VEL_X);
  T vel_y = state((int)StateIndex::VEL_Y);
  T vel_z = state((int)StateIndex::VEL_Z);
  T thrust = state((int)StateIndex::THRUST);

  T distance = state((int)StateIndex::DISTANCE);

  if (std::isnan(pos_x) ||
      std::isnan(pos_y) ||
      std::isnan(pos_z) ||
      std::isnan(rot_w) ||
      std::isnan(rot_x) ||
      std::isnan(rot_y) ||
      std::isnan(rot_z) ||
      std::isnan(vel_x) ||
      std::isnan(vel_y) ||
      std::isnan(vel_z) ||
      std::isnan(thrust) ||
      std::isnan(distance)) {
    return false;
  }

  predicted_state = vision_common::StateEstimate();
  predicted_state.timestamp = time;
  predicted_state.position.x() = pos_x;
  predicted_state.position.y() = pos_y;
  predicted_state.position.z() = pos_z;
  predicted_state.orientation.w() = rot_w;
  predicted_state.orientation.x() = rot_x;
  predicted_state.orientation.y() = rot_y;
  predicted_state.orientation.z() = rot_z;
  predicted_state.velocity.x() = vel_x;
  predicted_state.velocity.y() = vel_y;
  predicted_state.velocity.z() = vel_z;
  predicted_state.collective_thrust = thrust;

  predicted_state.gates.gates.resize(1);
  predicted_state.gates.gates[0].center.normal_vector.orientation.w() = state((int)StateIndex::FOC_W);
  predicted_state.gates.gates[0].center.normal_vector.orientation.x() = state((int)StateIndex::FOC_X);
  predicted_state.gates.gates[0].center.normal_vector.orientation.y() = state((int)StateIndex::FOC_Y);
  predicted_state.gates.gates[0].center.normal_vector.orientation.z() = state((int)StateIndex::FOC_Z);
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

//TODO: MPC 
template <typename T>
bool MpcController<T>::setNewParams(MpcParams<T>& params) {

  // mpc_wrapper_.setCosts(params.Q_, params.R_, params.state_cost_exponential_, params.input_cost_exponential_);
  
  mpc_wrapper_.setSv2ScaledCosts(params.Q_, params.R_, params.sv2_scalings_);

  mpc_wrapper_.setLimits(params.max_thrust_rate_,
                         params.max_bodyrate_x_,
                         params.max_bodyrate_y_,
                         params.max_bodyrate_z_,
                         params.min_thrust_,
                         params.max_thrust_);
  mpc_wrapper_.setCameraParameters(params.t_B_C_, params.q_B_C_);
  mpc_wrapper_.setFocusParameters(params.focus_p_, params.focus_q_);
  mpc_wrapper_.setGateParameters(params.gate_p_, params.gate_q_);
  for (int i = 0; i < kSamples+1; ++i) {
    mpc_wrapper_.setTrajectoryParameters(i, params.polynomial_trajectory_[i].poly_x,
                                            params.polynomial_trajectory_[i].poly_y,
                                            params.polynomial_trajectory_[i].poly_z,
                                            params.polynomial_trajectory_[i].distance_start);
  }   
  mpc_wrapper_.setSizeParameters(params.gate_width_, params.gate_height_, params.quad_radius_);

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