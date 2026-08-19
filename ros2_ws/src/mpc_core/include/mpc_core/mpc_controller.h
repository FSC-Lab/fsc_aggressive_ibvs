#pragma once

#include <thread>
#include <iostream>
#include <list>
#include <math.h>
#include <vector>

#include <Eigen/Eigen>

#include "mpc_core/logging.h"
#include "mpc_core/mpc_params.h"
#include "mpc_core/mpc_wrapper.h"
#include "mpc_core/types.h"

namespace mpc {

template<typename T>
class MpcController {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Velocity-body + pitch-gimbal model (utils/pose_track_mpc_gimbal.cpp).
  static_assert(kStateSize == 15,
                "MpcController: Wrong model size. Number of states does not match.");
  static_assert(kInputSize == 7,
                "MpcController: Wrong model size. Number of inputs does not match.");

  MpcController();
  ~MpcController();

  // Apply an already-configured parameter set and start the preparation thread.
  // Must be called once before run(). Returns false if the wrapper cannot be
  // prepared with the given parameters.
  bool initialize(MpcParams<T>& params);

  mpc_core::ControlCommand off();

  bool run(const mpc_core::StateEstimate& state_est,
           const mpc_core::Trajectory& reference_trajectory,
           MpcParams<T>& params,
           std::list<mpc_core::ControlCommand>& command_queue,
           std::vector<mpc_core::StateEstimate>& predicted_states);

  bool run(const mpc_core::StateEstimate& state_est,
           const mpc_core::Trajectory& reference_trajectory,
           MpcParams<T>& params,
           std::list<mpc_core::ControlCommand>& command_queue);

  void reset();

  // Feedback of the current gimbal pitch angle [rad] (camera tilt). If gimbal
  // encoder feedback is available, the node pushes it here; otherwise the
  // controller warm-starts from its own last prediction.
  void setGimbalAngle(const T gimbal_angle) { gimbal_angle_ = gimbal_angle; }

 private:
  bool setState(const mpc_core::StateEstimate& state_est);

  bool setReference(const mpc_core::Trajectory& reference_trajectory);

  bool getControlCommand(
      const Eigen::Ref<const Eigen::Matrix<T, kStateSize, 1>> state,
      const Eigen::Ref<const Eigen::Matrix<T, kInputSize, 1>> input,
      double& time, mpc_core::ControlCommand& command);

  bool getPredictedState(
      const Eigen::Ref<const Eigen::Matrix<T, kStateSize, 1>> state,
      double& time, mpc_core::StateEstimate& predicted_state);

  void preparationThread();

  bool setNewParams(MpcParams<T>& params);

  // Parameters
  MpcParams<T> params_;

  // MPC
  MpcWrapper<T> mpc_wrapper_;

  // Preparation Thread
  std::thread preparation_thread_;

  // Variables
  bool initialized_{false};
  T timing_feedback_, timing_preparation_;
  bool solve_from_scratch_;
  T gimbal_angle_{0.0};  // current gimbal pitch feedback [rad]
  Eigen::Matrix<T, kStateSize, 1> est_state_;
  Eigen::Matrix<T, kCostSize, kSamples + 1> reference_states_;
  Eigen::Matrix<T, kInputSize, kSamples + 1> reference_inputs_;
  Eigen::Matrix<T, kStateSize, kSamples + 1> predicted_states_;
  Eigen::Matrix<T, kInputSize, kSamples> predicted_inputs_;
};

}
