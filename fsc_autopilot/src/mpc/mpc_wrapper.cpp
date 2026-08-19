#include "mpc/mpc_wrapper.h"

namespace mpc {

template <typename T>
MpcWrapper<T>::MpcWrapper() {
  // Clear solver memory.
  memset(&acadoWorkspace, 0, sizeof(acadoWorkspace));
  memset(&acadoVariables, 0, sizeof(acadoVariables));

  // Initialize the solver.
  acado_initializeSolver();

  // Initialize the states and controls.
  // State (15): p(3), psi, v(3), f1(4), r1, theta_g, dy1, dy2
  const Eigen::Matrix<T, kStateSize, 1> hover_state =
      (Eigen::Matrix<T, kStateSize, 1>() << 0.0, 0.0, 0.0,
                                            0.0,                 // psi
                                            0.0, 0.0, 0.0,       // vel
                                            1.0, 0.0, 0.0, 0.0,  // bearing quaternion
                                            10.0,                // range
                                            0.0,                 // gimbal angle
                                            0.0, 0.0).finished();// slack integrators
  // Cost-state reference (14): p(3), psi, v(3), img_u, img_v, err(3), rdot1, theta_g
  const Eigen::Matrix<T, kCostSize, 1> hover_ref =
      (Eigen::Matrix<T, kCostSize, 1>() << 0.0, 0.0, 0.0,
                                           0.0,            // psi
                                           0.0, 0.0, 0.0,  // vel
                                           0.0, 0.0,       // img_uv
                                           0.0, 0.0, 0.0,  // err
                                           0.0,            // distance rate
                                           0.0).finished();// gimbal angle

  // Initialize states x and xN and input u.
  acado_initial_state_ = hover_state.template cast<float>();

  acado_states_ = hover_state.replicate(1, kSamples + 1).template cast<float>();

  acado_inputs_ = kHoverInput_.replicate(1, kSamples).template cast<float>();

  // Initialize references y and yN.
  acado_reference_states_.block(0, 0, kCostSize, kSamples) =
      hover_ref.replicate(1, kSamples).template cast<float>();

  // kCostSize == kStateSize
  acado_reference_states_.block(kCostSize, 0, kInputSize, kSamples) =
      kHoverInput_.replicate(1, kSamples);

  acado_reference_end_state_.segment(0, kCostSize) =
      hover_ref.template cast<float>();

  // Initialize Cost matrix W and WN.
  if (!(acado_W_.trace() > 0.0)) {
    acado_W_ = W_.replicate(1, kSamples).template cast<float>();
    acado_W_end_ = WN_.template cast<float>();
  }
  // Initialize online data.
  //TODO: MPC  
  Eigen::Matrix<T, 3, 1> p_init(0, 0, 0);
  Eigen::Quaternion<T> q_init(1, 0, 0, 0);
  setCameraParameters(p_init, q_init);
  setFocusParameters(p_init);
  setTrajectoryParameters(Eigen::Matrix<T, 4, 1>::Zero(),
                          Eigen::Matrix<T, 4, 1>::Zero(),
                          Eigen::Matrix<T, 4, 1>::Zero());

  // Initialize solver.
  acado_initializeNodesByForwardSimulation();
  acado_preparationStep();
  acado_is_prepared_ = true;

  std::cout << "kStateSize: " << kStateSize << std::endl;
  std::cout << "kInputSize: " << kInputSize << std::endl;
  std::cout << "kCostSize: " << kCostSize << std::endl;
  std::cout << "kSamples: " << kSamples << std::endl;
  std::cout << "kOdSize: " << kOdSize << std::endl;
  std::cout << "kRefSize: " << kRefSize << std::endl;
}

// Constructor with cost matrices as arguments.
template <typename T>
MpcWrapper<T>::MpcWrapper(
    const Eigen::Ref<const Eigen::Matrix<T, kCostSize, kCostSize>> Q,
    const Eigen::Ref<const Eigen::Matrix<T, kInputSize, kInputSize>> R) {
  setCosts(Q, R);
  MpcWrapper();
}

// Set cost matrices with optional scaling.
template <typename T>
bool MpcWrapper<T>::setCosts(
    const Eigen::Ref<const Eigen::Matrix<T, kCostSize, kCostSize>> Q,
    const Eigen::Ref<const Eigen::Matrix<T, kInputSize, kInputSize>> R,
    const T state_cost_scaling, const T input_cost_scaling) {
  if (state_cost_scaling < 0.0 || input_cost_scaling < 0.0) {
    ROS_ERROR("MPC: Cost scaling is wrong, must be non-negative!");
    return false;
  }

  float state_scale{1.0};
  float input_scale{1.0};
  for (int i = 0; i < kSamples; i++) {
    state_scale = exp(-float(i) / float(kSamples) * float(state_cost_scaling));
    input_scale = exp(-float(i) / float(kSamples) * float(input_cost_scaling));
    setCost(i, Q*state_scale, R*input_scale);
  }

  return true;
}

template <typename T>
bool MpcWrapper<T>::setCost(
    const int node_index,
    const Eigen::Ref<const Eigen::Matrix<T, kCostSize, kCostSize>> Q,
    const Eigen::Ref<const Eigen::Matrix<T, kInputSize, kInputSize>> R) {
  if (node_index > kSamples-1) {
    return false;
  }

  W_.block(0, 0, kCostSize, kCostSize) = Q;
  W_.block(kCostSize, kCostSize, kInputSize, kInputSize) = R;
  acado_W_.block(0, node_index * kRefSize, kCostSize, kCostSize) = W_.block(0, 0, kCostSize, kCostSize).template cast<float>();
  acado_W_.block(kCostSize, node_index * kRefSize + kCostSize, kInputSize, kInputSize) = W_.block(kCostSize, kCostSize, kInputSize, kInputSize).template cast<float>();
  
  // last sample costs
  if (node_index == kSamples-1) {
    WN_ = W_.block(0, 0, kCostSize, kCostSize);
    acado_W_end_ = WN_.template cast<float>();
  }

  return true;
}

template <typename T>
bool MpcWrapper<T>::setSv1ScaledCosts(
    const Eigen::Ref<const Eigen::Matrix<T, kCostSize, kCostSize>> Q,
    const Eigen::Ref<const Eigen::Matrix<T, kInputSize, kInputSize>> R,
    const std::vector<T> sv1_scalings) {
  for (int i = 0; i < kSamples; i++) {
    Eigen::Matrix<T, kInputSize, kInputSize> R_scaled;
    R_scaled = (Eigen::Matrix<T, kInputSize, 1>()
        << R.diagonal()[0],
           R.diagonal()[1],
           R.diagonal()[2],
           R.diagonal()[3],
           R.diagonal()[4],
           std::max(float(R.diagonal()[5]*sv1_scalings[i]), 0.001f),
           R.diagonal()[6]).finished().asDiagonal();

    setCost(i, Q, R_scaled);
  }
  return true;
}

template <typename T>
bool MpcWrapper<T>::setSv2ScaledCosts(
    const Eigen::Ref<const Eigen::Matrix<T, kCostSize, kCostSize>> Q,
    const Eigen::Ref<const Eigen::Matrix<T, kInputSize, kInputSize>> R,
    const std::vector<T> sv2_scalings) {
  for (int i = 0; i < kSamples; i++) {
    Eigen::Matrix<T, kInputSize, kInputSize> R_scaled;
    R_scaled = (Eigen::Matrix<T, kInputSize, 1>()
        << R.diagonal()[0],
           R.diagonal()[1],
           R.diagonal()[2],
           R.diagonal()[3],
           R.diagonal()[4],
           R.diagonal()[5],
           std::max(float(R.diagonal()[6]*sv2_scalings[i]), 0.001f)).finished().asDiagonal();

    setCost(i, Q, R_scaled);
  }
  return true;
}



// Set the input limits (velocity body + pitch gimbal).
template <typename T>
bool MpcWrapper<T>::setLimits(
    T max_velocity,
    T max_yaw_rate,
    T max_gimbal_rate) {

  if (max_velocity <= 0.0 || max_yaw_rate <= 0.0 || max_gimbal_rate <= 0.0) {
    ROS_ERROR("MPC: Maximal values are not set properly.");
    return false;
  }

  // Set input boundaries.
  //   [vx_cmd, vy_cmd, vz_cmd, psi_rate, omega_g, sv1, sv2]
  Eigen::Matrix<T, kInputSize, 1> lower_bounds = Eigen::Matrix<T, kInputSize, 1>::Zero();
  Eigen::Matrix<T, kInputSize, 1> upper_bounds = Eigen::Matrix<T, kInputSize, 1>::Zero();

  lower_bounds << -max_velocity, -max_velocity, -max_velocity,
                  -max_yaw_rate, -max_gimbal_rate, 0.0, 0.0;
  upper_bounds << max_velocity, max_velocity, max_velocity,
                  max_yaw_rate, max_gimbal_rate, 1000.0, 1000.0;

  acado_lower_bounds_ =
      lower_bounds.replicate(1, kSamples).template cast<float>();
  acado_upper_bounds_ =
      upper_bounds.replicate(1, kSamples).template cast<float>();

  return true;
}

// Set camera mount: body -> gimbal-base translation + rotation.
template <typename T>
bool MpcWrapper<T>::setCameraParameters(
    const Eigen::Ref<const Eigen::Matrix<T, 3, 1>>& t_B_C,
    const Eigen::Quaternion<T>& q_B_M) {
  acado_online_data_.block(OnlineDataIndex::CAM_T, 0, 3, ACADO_N + 1) =
      t_B_C.replicate(1, ACADO_N + 1).template cast<float>();

  Eigen::Matrix<T, 4, 1> q_B_M_mat(q_B_M.w(), q_B_M.x(), q_B_M.y(), q_B_M.z());
  acado_online_data_.block(OnlineDataIndex::MOUNT_R, 0, 4, ACADO_N + 1) =
      q_B_M_mat.replicate(1, ACADO_N + 1).template cast<float>();

  return true;
}

// Set the target position anchor (world frame).
template <typename T>
bool MpcWrapper<T>::setFocusParameters(
    const Eigen::Ref<const Eigen::Matrix<T, 3, 1>>& focus_p) {
  acado_online_data_.block(OnlineDataIndex::FOC_P, 0, 3, ACADO_N + 1) =
      focus_p.replicate(1, ACADO_N + 1).template cast<float>();

  return true;
}

template <typename T>
bool MpcWrapper<T>::setTrajectoryParameters(
    const Eigen::Ref<const Eigen::Matrix<T, 4, 1>>& poly_x,
    const Eigen::Ref<const Eigen::Matrix<T, 4, 1>>& poly_y,
    const Eigen::Ref<const Eigen::Matrix<T, 4, 1>>& poly_z,
    const T distance_start) {

  acado_online_data_.block(OnlineDataIndex::POLY_X, 0, 4, ACADO_N + 1) =
      poly_x.replicate(1, ACADO_N + 1).template cast<float>();

  acado_online_data_.block(OnlineDataIndex::POLY_Y, 0, 4, ACADO_N + 1) =
      poly_y.replicate(1, ACADO_N + 1).template cast<float>();

  acado_online_data_.block(OnlineDataIndex::POLY_Z, 0, 4, ACADO_N + 1) =
      poly_z.replicate(1, ACADO_N + 1).template cast<float>();

  acado_online_data_.block(OnlineDataIndex::DISTANCE_START, 0, 1, ACADO_N + 1) =
      (Eigen::Matrix<T, 1, 1>() << distance_start).finished().replicate(1, ACADO_N + 1).template cast<float>();


  return true;
}

template <typename T>
bool MpcWrapper<T>::setTrajectoryParameters(
    const int node_index,
    const Eigen::Ref<const Eigen::Matrix<T, 4, 1>>& poly_x,
    const Eigen::Ref<const Eigen::Matrix<T, 4, 1>>& poly_y,
    const Eigen::Ref<const Eigen::Matrix<T, 4, 1>>& poly_z,
    const T distance_start) {
  
  if (node_index > kSamples) {
    return false;
  }

  acado_online_data_.block(OnlineDataIndex::POLY_X, node_index, 4, 1) = poly_x.template cast<float>();
  acado_online_data_.block(OnlineDataIndex::POLY_Y, node_index, 4, 1) = poly_y.template cast<float>();
  acado_online_data_.block(OnlineDataIndex::POLY_Z, node_index, 4, 1) = poly_z.template cast<float>();
  acado_online_data_.block(OnlineDataIndex::DISTANCE_START, node_index, 1, 1) << static_cast<float>(distance_start);

  return true;
}

// Set a reference pose.
template <typename T>
bool MpcWrapper<T>::setReferencePose(
    const Eigen::Ref<const Eigen::Matrix<T, kCostSize, 1>> reference) {
  acado_reference_states_.block(0, 0, kCostSize, kSamples) =
      reference.replicate(1, kSamples).template cast<float>();

  acado_reference_states_.block(kCostSize, 0, kInputSize, kSamples) =
      kHoverInput_.replicate(1, kSamples);

  acado_reference_end_state_.segment(0, kCostSize) =
      reference.template cast<float>();

  acado_initializeNodesByForwardSimulation();
  return true;
}

// Set a reference trajectory.
template <typename T>
bool MpcWrapper<T>::setTrajectory(
    const Eigen::Ref<const Eigen::Matrix<T, kCostSize, kSamples + 1>> references,
    const Eigen::Ref<const Eigen::Matrix<T, kInputSize, kSamples + 1>> inputs) {
  Eigen::Map<Eigen::Matrix<float, kRefSize, kSamples, Eigen::ColMajor>> y(
      const_cast<float*>(acadoVariables.y));

  acado_reference_states_.block(0, 0, kCostSize, kSamples) =
      references.block(0, 0, kCostSize, kSamples).template cast<float>();

  acado_reference_states_.block(kCostSize, 0, kInputSize, kSamples) =
      inputs.block(0, 0, kInputSize, kSamples).template cast<float>();

  acado_reference_end_state_.segment(0, kCostSize) =
      references.col(kSamples).template cast<float>();

  return true;
}

// Reset states and inputs and calculate new solution.
template <typename T>
bool MpcWrapper<T>::solve(
    const Eigen::Ref<const Eigen::Matrix<T, kStateSize, 1>> state) {
  acado_states_ = state.replicate(1, kSamples + 1).template cast<float>();

  acado_inputs_ = kHoverInput_.replicate(1, kSamples);

  return update(state);
}

// Calculate new solution from last known solution.
template <typename T>
bool MpcWrapper<T>::update(
    const Eigen::Ref<const Eigen::Matrix<T, kStateSize, 1>> state,
    bool do_preparation) {
  if (!acado_is_prepared_) {
    ROS_WARN("MPC: Solver was triggered without preparation, abort!");
    return false;
  }

  // Velocity-body model: heading is a plain angle, no quaternion hemisphere
  // flip needed (the body attitude quaternion is no longer a state).
  acado_initial_state_ = state.template cast<float>();

  // Perform feedback step and reset preparation check.
  acado_feedbackStep();
  acado_is_prepared_ = false;

  // Prepare if the solver if wanted
  if (do_preparation) {
    acado_preparationStep();
    acado_is_prepared_ = true;
  }

  return true;
}

// Prepare the solver.
// Must be triggered between iterations if not done in the update function.
template <typename T>
bool MpcWrapper<T>::prepare() {
  acado_preparationStep();
  acado_is_prepared_ = true;

  return true;
}

// Get a specific state.
template <typename T>
void MpcWrapper<T>::getState(
    const int node_index,
    Eigen::Ref<Eigen::Matrix<T, kStateSize, 1>> return_state) {
  return_state = acado_states_.col(node_index).cast<T>();
}

// Get all states.
template <typename T>
void MpcWrapper<T>::getStates(
    Eigen::Ref<Eigen::Matrix<T, kStateSize, kSamples + 1>> return_states) {
  return_states = acado_states_.cast<T>();
}

// Get a specific input.
template <typename T>
void MpcWrapper<T>::getInput(
    const int node_index,
    Eigen::Ref<Eigen::Matrix<T, kInputSize, 1>> return_input) {
  return_input = acado_inputs_.col(node_index).cast<T>();
}

// Get all inputs.
template <typename T>
void MpcWrapper<T>::getInputs(
    Eigen::Ref<Eigen::Matrix<T, kInputSize, kSamples>> return_inputs) {
  return_inputs = acado_inputs_.cast<T>();
}

template class MpcWrapper<float>;
template class MpcWrapper<double>;

}  // namespace mpc
