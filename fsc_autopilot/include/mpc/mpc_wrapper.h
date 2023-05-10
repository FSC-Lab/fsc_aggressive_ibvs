#pragma once

#include <Eigen/Eigen>
#include <ros/ros.h>
#include <iostream>

namespace mpc {

#include "acado_auxiliary_functions.h"
#include "acado_common.h"

static constexpr int kSamples = ACADO_N;      // number of samples
static constexpr int kStateSize = ACADO_NX;   // number of states
static constexpr int kRefSize = ACADO_NY;     // number of reference states
static constexpr int kEndRefSize = ACADO_NYN; // number of end reference states
static constexpr int kInputSize = ACADO_NU;   // number of inputs
static constexpr int kCostSize = ACADO_NY - ACADO_NU; // number of costs
static constexpr int kOdSize = ACADO_NOD;     // number of online data

ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

//TODO: MPC
enum class StateIndex {
  POS_X = 0,
  POS_Y = 1,
  POS_Z = 2,
  ROT_W = 3,
  ROT_X = 4,
  ROT_Y = 5,
  ROT_Z = 6,
  VEL_X = 7,
  VEL_Y = 8,
  VEL_Z = 9,
  THRUST = 10,
  FOC_W = 11,
  FOC_X = 12,
  FOC_Y = 13,
  FOC_Z = 14,
  DISTANCE = 15,
  SLA_1 = 16,
  SLA_2 = 17,
};

//TODO: MPC
enum class ReferenceIndex {
  POS_X = 0,
  POS_Y = 1,
  POS_Z = 2,
  ROT_W = 3,
  ROT_X = 4,
  ROT_Y = 5,
  ROT_Z = 6,
  VEL_X = 7,
  VEL_Y = 8,
  VEL_Z = 9,
  THRUST = 10,
  IMG_U = 11,
  IMG_V = 12,
  ERR_X = 13, 
  ERR_Y = 14,
  ERR_Z = 15,
  DISTANCE_RATE = 16,
};

//TODO: MPC
enum class ControlIndex {
  THRUST_DOT = 0,
  RAT_X = 1,
  RAT_Y = 2,
  RAT_Z = 3,
  SLA_1 = 4,
  SLA_2 = 5,
};

//TODO: MPC
enum OnlineDataIndex {
  CAM_T = 0,
  CAM_R = 3,
  FOC_P = 7,
  FOC_R = 10,
  BOX_P = 14,
  BOX_R = 17,
  POLY_X = 21,
  POLY_Y = 25,
  POLY_Z = 29,
  DISTANCE_START = 33,
  BOX_W = 34,
  BOX_H = 35,
  QUAD_RADIUS = 36
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

  //TODO: MPC
  bool setLimits(T max_thrust_rate,
                 T max_rollrate,
                 T max_pitchrate,
                 T max_yawrate,
                 T min_thrust,
                 T max_thrust);

  bool setCameraParameters(
    const Eigen::Ref<const Eigen::Matrix<T, 3, 1>>& t_B_C,
    const Eigen::Quaternion<T>& q_B_C);

  //TODO: MPC
  bool setFocusParameters(
    const Eigen::Ref<const Eigen::Matrix<T, 3, 1>>& focus_p,
    const Eigen::Quaternion<T>& focus_q);
  //TODO: MPC
  bool setGateParameters(
    const Eigen::Ref<const Eigen::Matrix<T, 3, 1>>& gate_p,
    const Eigen::Quaternion<T>& gate_q);
  //TODO: MPC
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
  //TODO: MPC
  bool setSizeParameters(const T gate_width, const T gate_height, const T quad_radius);

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

  //TODO: MPC
  //TODO: remember to increase input size here
  Eigen::Map<Eigen::Matrix<float, 6, kSamples, Eigen::ColMajor>>
    acado_lower_bounds_{acadoVariables.lbValues};

  Eigen::Map<Eigen::Matrix<float, 6, kSamples, Eigen::ColMajor>>
    acado_upper_bounds_{acadoVariables.ubValues};

  //TODO: MPC
  // px,py,pz,qw,qx,qy,qz,vx,vy,vz,Th
  Eigen::Matrix<T, kRefSize, kRefSize> W_ = (Eigen::Matrix<T, kRefSize, 1>() <<
    100 * Eigen::Matrix<T, 3, 1>::Ones(), // pos
    100 * Eigen::Matrix<T, 4, 1>::Ones(), // rot
    10 * Eigen::Matrix<T, 3, 1>::Ones(),  // vel
    1 * Eigen::Matrix<T, 1, 1>::Ones(),   // thrust
    0 * Eigen::Matrix<T, 2, 1>::Ones(), // img_uv
    0 * Eigen::Matrix<T, 3, 1>::Ones(), // err
    0 * Eigen::Matrix<T, 1, 1>::Ones(), // distance rate
    1, 1, 1, 1, 1, 1).finished().asDiagonal();

  Eigen::Matrix<T, kEndRefSize, kEndRefSize> WN_ =
    W_.block(0, 0, kEndRefSize, kEndRefSize);

  bool acado_is_prepared_{false};

  const T dt_{0.05};

  //TODO: MPC
  const Eigen::Matrix<real_t, kInputSize, 1> kHoverInput_ =
    (Eigen::Matrix<real_t, kInputSize, 1>() << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0).finished();
};

}