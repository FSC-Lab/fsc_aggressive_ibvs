#pragma once

#include <Eigen/Eigen>
#include <iostream>
#include <vector>

#include "mpc_core/logging.h"

namespace mpc {

#include "acado_auxiliary_functions.h"
#include "acado_common.h"

// NOTE: acado_common.h already declares (extern "C")
//   extern ACADOvariables acadoVariables;
//   extern ACADOworkspace acadoWorkspace;
// The single definition of these globals lives in mpc_wrapper.cpp. (The ROS1
// code defined them here in the header and relied on the compiler's legacy
// -fcommon tentative-definition merging; modern toolchains default to
// -fno-common, so we give them exactly one definition instead.)

static constexpr int kSamples = ACADO_N;      // number of samples
static constexpr int kStateSize = ACADO_NX;   // number of states
static constexpr int kRefSize = ACADO_NY;     // number of reference states
static constexpr int kEndRefSize = ACADO_NYN; // number of end reference states
static constexpr int kInputSize = ACADO_NU;   // number of inputs
static constexpr int kCostSize = ACADO_NY - ACADO_NU; // number of costs
static constexpr int kOdSize = ACADO_NOD;     // number of online data

// State layout (15) for the velocity-body + pitch-gimbal model, matching
// utils/pose_track_mpc_gimbal.cpp:
//   [p(3), psi, v(3), f1(4), r1, theta_g, dy1, dy2]
enum class StateIndex {
  POS_X = 0,
  POS_Y = 1,
  POS_Z = 2,
  PSI = 3,          // body heading (camera pan)
  VEL_X = 4,
  VEL_Y = 5,
  VEL_Z = 6,
  BEA_W = 7,        // bearing (line-of-sight) quaternion f1
  BEA_X = 8,
  BEA_Y = 9,
  BEA_Z = 10,
  DISTANCE = 11,    // range r1 to target
  GIMBAL = 12,      // gimbal pitch angle theta_g (camera tilt)
  SLA_1 = 13,       // slack integrator dy1
  SLA_2 = 14,       // slack integrator dy2
};

// Cost / reference layout -- state-cost part only (14), matching hN in the model:
//   [p(3), psi, v(3), img_u, img_v, err(3), rdot1, theta_g]
enum class ReferenceIndex {
  POS_X = 0,
  POS_Y = 1,
  POS_Z = 2,
  PSI = 3,
  VEL_X = 4,
  VEL_Y = 5,
  VEL_Z = 6,
  IMG_U = 7,
  IMG_V = 8,
  ERR_X = 9,
  ERR_Y = 10,
  ERR_Z = 11,
  DISTANCE_RATE = 12,
  GIMBAL = 13,
};

// Control layout (7):
//   [vx_cmd, vy_cmd, vz_cmd, psi_rate, omega_g, sv1, sv2]
enum class ControlIndex {
  VEL_X = 0,
  VEL_Y = 1,
  VEL_Z = 2,
  YAW_RATE = 3,
  GIMBAL_RATE = 4,
  SLA_1 = 5,
  SLA_2 = 6,
};

// Online-data layout (23):
//   [t_B_C(3), q_B_M(4), focus_p(3), poly_x(4), poly_y(4), poly_z(4), r_start(1)]
enum OnlineDataIndex {
  CAM_T = 0,          // body -> gimbal-mount translation
  MOUNT_R = 3,        // body -> gimbal-mount rotation quaternion q_B_M
  FOC_P = 7,          // target position anchor (world)
  POLY_X = 10,
  POLY_Y = 14,
  POLY_Z = 18,
  DISTANCE_START = 22
};

template <typename T>
class MpcWrapper {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  MpcWrapper();
  MpcWrapper(
    const Eigen::Ref<const Eigen::Matrix<T, kCostSize, kCostSize>> Q,
    const Eigen::Ref<const Eigen::Matrix<T, kInputSize, kInputSize>> R);

  bool setCosts(
    const Eigen::Ref<const Eigen::Matrix<T, kCostSize, kCostSize>> Q,
    const Eigen::Ref<const Eigen::Matrix<T, kInputSize, kInputSize>> R,
    const T state_cost_scaling = 0.0, const T input_cost_scaling = 0.0);

  bool setCost(
    const int node_index,
    const Eigen::Ref<const Eigen::Matrix<T, kCostSize, kCostSize>> Q,
    const Eigen::Ref<const Eigen::Matrix<T, kInputSize, kInputSize>> R);

  bool setSv1ScaledCosts(
    const Eigen::Ref<const Eigen::Matrix<T, kCostSize, kCostSize>> Q,
    const Eigen::Ref<const Eigen::Matrix<T, kInputSize, kInputSize>> R,
    const std::vector<T> sv1_scalings);

  bool setSv2ScaledCosts(
    const Eigen::Ref<const Eigen::Matrix<T, kCostSize, kCostSize>> Q,
    const Eigen::Ref<const Eigen::Matrix<T, kInputSize, kInputSize>> R,
    const std::vector<T> sv2_scalings);

  // Velocity-body + gimbal input limits.
  bool setLimits(T max_velocity,
                 T max_yaw_rate,
                 T max_gimbal_rate);

  // Camera mount: body -> gimbal-base translation (t_B_C) and rotation (q_B_M).
  bool setCameraParameters(
    const Eigen::Ref<const Eigen::Matrix<T, 3, 1>>& t_B_C,
    const Eigen::Quaternion<T>& q_B_M);

  // Target position anchor (world) used to reconstruct the drone position.
  bool setFocusParameters(
    const Eigen::Ref<const Eigen::Matrix<T, 3, 1>>& focus_p);

  bool setTrajectoryParameters(
    const Eigen::Ref<const Eigen::Matrix<T, 4, 1>>& poly_x,
    const Eigen::Ref<const Eigen::Matrix<T, 4, 1>>& poly_y,
    const Eigen::Ref<const Eigen::Matrix<T, 4, 1>>& poly_z,
    const T distance_start = 0.0);
  //TODO: MPC
  bool setTrajectoryParameters(
    const int node_index,
    const Eigen::Ref<const Eigen::Matrix<T, 4, 1>>& poly_x,
    const Eigen::Ref<const Eigen::Matrix<T, 4, 1>>& poly_y,
    const Eigen::Ref<const Eigen::Matrix<T, 4, 1>>& poly_z,
    const T distance_start = 0.0);

  bool setReferencePose(
    const Eigen::Ref<const Eigen::Matrix<T, kCostSize, 1>> reference);

  bool setTrajectory(
    const Eigen::Ref<const Eigen::Matrix<T, kCostSize, kSamples+1>> references,
    const Eigen::Ref<const Eigen::Matrix<T, kInputSize, kSamples+1>> inputs);

  bool solve(const Eigen::Ref<const Eigen::Matrix<T, kStateSize, 1>> state);

  bool update(const Eigen::Ref<const Eigen::Matrix<T, kStateSize, 1>> state,
              bool do_preparation = true);
  bool prepare();

  void getState(const int node_index,
    Eigen::Ref<Eigen::Matrix<T, kStateSize, 1>> return_state);

  void getStates(
    Eigen::Ref<Eigen::Matrix<T, kStateSize, kSamples+1>> return_states);

  void getInput(const int node_index,
    Eigen::Ref<Eigen::Matrix<T, kInputSize, 1>> return_input);

  void getInputs(
    Eigen::Ref<Eigen::Matrix<T, kInputSize, kSamples>> return_input);

  T getTimestep() { return dt_; }

 private:
  Eigen::Map<Eigen::Matrix<float, kRefSize, kSamples, Eigen::ColMajor>>
    acado_reference_states_{acadoVariables.y};

  Eigen::Map<Eigen::Matrix<float, kEndRefSize, 1, Eigen::ColMajor>>
    acado_reference_end_state_{acadoVariables.yN};

  Eigen::Map<Eigen::Matrix<float, kStateSize, 1, Eigen::ColMajor>>
    acado_initial_state_{acadoVariables.x0};

  Eigen::Map<Eigen::Matrix<float, kStateSize, kSamples+1, Eigen::ColMajor>>
    acado_states_{acadoVariables.x};

  Eigen::Map<Eigen::Matrix<float, kInputSize, kSamples, Eigen::ColMajor>>
    acado_inputs_{acadoVariables.u};

  Eigen::Map<Eigen::Matrix<float, kOdSize, kSamples+1, Eigen::ColMajor>>
    acado_online_data_{acadoVariables.od};

  Eigen::Map<Eigen::Matrix<float, kRefSize, kRefSize * kSamples>>
    acado_W_{acadoVariables.W};

  Eigen::Map<Eigen::Matrix<float, kEndRefSize, kEndRefSize>>
    acado_W_end_{acadoVariables.WN};

  Eigen::Map<Eigen::Matrix<float, kInputSize, kSamples, Eigen::ColMajor>>
    acado_lower_bounds_{acadoVariables.lbValues};

  Eigen::Map<Eigen::Matrix<float, kInputSize, kSamples, Eigen::ColMajor>>
    acado_upper_bounds_{acadoVariables.ubValues};

  // Cost weights, ordered as the model cost outputs h (21):
  //   p(3), psi, v(3), img_u, img_v, err(3), rdot1, theta_g,
  //   vx_cmd, vy_cmd, vz_cmd, psi_rate, omega_g, sv1, sv2
  Eigen::Matrix<T, kRefSize, kRefSize> W_ = (Eigen::Matrix<T, kRefSize, 1>() <<
    100 * Eigen::Matrix<T, 3, 1>::Ones(), // pos
    50 * Eigen::Matrix<T, 1, 1>::Ones(),  // yaw (psi)
    10 * Eigen::Matrix<T, 3, 1>::Ones(),  // vel
    1 * Eigen::Matrix<T, 2, 1>::Ones(),   // img_uv
    1 * Eigen::Matrix<T, 3, 1>::Ones(),   // err
    1 * Eigen::Matrix<T, 1, 1>::Ones(),   // distance rate
    1 * Eigen::Matrix<T, 1, 1>::Ones(),   // gimbal angle
    1, 1, 1,                              // vel cmd
    1, 1,                                 // yaw rate, gimbal rate
    1, 1).finished().asDiagonal();        // slacks

  Eigen::Matrix<T, kEndRefSize, kEndRefSize> WN_ =
    W_.block(0, 0, kEndRefSize, kEndRefSize);

  bool acado_is_prepared_{false};

  const T dt_{0.05};

  // Neutral input: zero velocity / yaw-rate / gimbal-rate / slacks.
  const Eigen::Matrix<real_t, kInputSize, 1> kHoverInput_ =
    (Eigen::Matrix<real_t, kInputSize, 1>() <<
       0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0).finished();
};

}
