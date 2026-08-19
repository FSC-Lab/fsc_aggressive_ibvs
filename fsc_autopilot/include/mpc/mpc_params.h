#pragma once

#include <mpc/mpc_wrapper.h>
#include <ros/ros.h>

#include "vision_common/parameter_helper.h"
#include "vision_common/polynomial_trajectory.h"

namespace mpc {

template <typename T>
struct PolynomialParams {
  PolynomialParams()
    : poly_x(Eigen::Matrix<T, 4, 1>::Zero()),
      poly_y(Eigen::Matrix<T, 4, 1>::Zero()),
      poly_z(Eigen::Matrix<T, 4, 1>::Zero()),
      distance_start(0.0) {}
  Eigen::Matrix<T, 4, 1> poly_x;
  Eigen::Matrix<T, 4, 1> poly_y;
  Eigen::Matrix<T, 4, 1> poly_z;
  T distance_start;
};

// Parameters for the VELOCITY-body + 1-axis PITCH-gimbal MPC variant.
// Cost layout (kCostSize == 14): p(3), psi, v(3), img_u, img_v, err(3),
//                                distance_rate, theta_g
// Input layout (kInputSize == 7): vx, vy, vz, psi_rate, omega_g, sv1, sv2
template <typename T>
class MpcParams {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  MpcParams()
      : changed_(false),
        print_info_(false),
        state_cost_exponential_(0.0),
        input_cost_exponential_(0.0),
        max_velocity_(0.0),
        max_yaw_rate_(0.0),
        max_gimbal_rate_(0.0),
        gimbal_angle_min_(0.0),
        gimbal_angle_max_(0.0),
        t_B_C_(Eigen::Matrix<T, 3, 1>::Zero()),
        q_B_M_(Eigen::Quaternion<T>(1.0, 0.0, 0.0, 0.0)),
        focus_p_(Eigen::Matrix<T, 3, 1>::Zero()),
        Q_(Eigen::Matrix<T, kCostSize, kCostSize>::Zero()),
        R_(Eigen::Matrix<T, kInputSize, kInputSize>::Zero()) {
    polynomial_trajectory_.resize(kSamples + 1);
    sv1_scalings_ = std::vector<T>(kSamples, 1.0);
    sv2_scalings_ = std::vector<T>(kSamples, 1.0);
  }

  ~MpcParams() {}

  void setPositionWeight(const double Q_pos_xy, const double Q_pos_z) {
    Q_.block((int)ReferenceIndex::POS_X, (int)ReferenceIndex::POS_X, 3, 3) =
        (Eigen::Matrix<T, 3, 1>() << Q_pos_xy, Q_pos_xy, Q_pos_z).finished().asDiagonal();

    Q_pos_xy_ = Q_pos_xy;
    Q_pos_z_ = Q_pos_z;

    changed_ = true;
  }

  void setYawWeight(const double Q_yaw) {
    Q_.block((int)ReferenceIndex::PSI, (int)ReferenceIndex::PSI, 1, 1) =
        (Eigen::Matrix<T, 1, 1>() << Q_yaw).finished().asDiagonal();

    Q_yaw_ = Q_yaw;

    changed_ = true;
  }

  void setVelocityWeight(const double Q_vel) {
    Q_.block((int)ReferenceIndex::VEL_X, (int)ReferenceIndex::VEL_X, 3, 3) = (Eigen::Matrix<T, 3, 1>()
        << Q_vel, Q_vel, Q_vel).finished().asDiagonal();

    Q_vel_ = Q_vel;

    changed_ = true;
  }

  void setVisualServoWeight(const double Q_err) {
    Q_.block((int)ReferenceIndex::ERR_X, (int)ReferenceIndex::ERR_X, 3, 3) = (Eigen::Matrix<T, 3, 1>()
        << Q_err, Q_err, Q_err).finished().asDiagonal();

    Q_err_ = Q_err;

    changed_ = true;
  }

  void setPerceptionWeight(
      const double Q_img_u,
      const double Q_img_v) {
    Q_.block((int)ReferenceIndex::IMG_U, (int)ReferenceIndex::IMG_U, 2, 2) = (Eigen::Matrix<T, 2, 1>()
        << Q_img_u, Q_img_v).finished().asDiagonal();

    Q_img_u_ = Q_img_u;
    Q_img_v_ = Q_img_v;

    changed_ = true;
  }

  void setReferenceSpeed(
      const double Q_drate) {
    Q_.block((int)ReferenceIndex::DISTANCE_RATE, (int)ReferenceIndex::DISTANCE_RATE, 1, 1) = (Eigen::Matrix<T, 1, 1>()
        << Q_drate).finished().asDiagonal();

    Q_drate_ = Q_drate;

    changed_ = true;
  }

  void setGimbalWeight(const double Q_gimbal) {
    Q_.block((int)ReferenceIndex::GIMBAL, (int)ReferenceIndex::GIMBAL, 1, 1) =
        (Eigen::Matrix<T, 1, 1>() << Q_gimbal).finished().asDiagonal();

    Q_gimbal_ = Q_gimbal;

    changed_ = true;
  }

  void setSlackVariableWeight(
      const double R_sv1,
      const double R_sv2) {
    R_.block((int)ControlIndex::SLA_1, (int)ControlIndex::SLA_1, 2, 2) = (Eigen::Matrix<T, 2, 1>()
        << R_sv1, R_sv2).finished().asDiagonal();

    R_sv1_ = R_sv1;
    R_sv2_ = R_sv2;

    changed_ = true;
  }

  void setSv1Weight(
      const double R_sv1) {
    R_.block((int)ControlIndex::SLA_1, (int)ControlIndex::SLA_1, 1, 1) = (Eigen::Matrix<T, 1, 1>()
        << R_sv1).finished().asDiagonal();

    R_sv1_ = R_sv1;

    changed_ = true;
  }

  void setSv2Weight(
      const double R_sv2) {
    R_.block((int)ControlIndex::SLA_2, (int)ControlIndex::SLA_2, 1, 1) = (Eigen::Matrix<T, 1, 1>()
        << R_sv2).finished().asDiagonal();

    R_sv2_ = R_sv2;

    changed_ = true;
  }

  void setSv1Scale(const int node_index, const double scale) {
    if (node_index > kSamples - 1) {
      return;
    }

    if (scale < 1e-6) {
      sv1_scalings_[node_index] = 1e-6;
      changed_ = true;
      return;
    }

    sv1_scalings_[node_index] = scale;
    changed_ = true;
  }

  void setSv2Scale(const int node_index, const double scale) {
    if (node_index > kSamples - 1) {
      return;
    }

    if (scale < 1e-6) {
      sv2_scalings_[node_index] = 1e-6;
      changed_ = true;
      return;
    }

    sv2_scalings_[node_index] = scale;
    changed_ = true;
  }

  // Camera mount: body -> gimbal-base translation (t_B_C) and rotation (q_B_M).
  void setCameraExtrinsics(const Eigen::Vector3d& t_B_C,
                           const Eigen::Quaterniond& q_B_M) {
    t_B_C_ = Eigen::Matrix<T, 3, 1>(t_B_C[0], t_B_C[1], t_B_C[2]);
    q_B_M_ = Eigen::Quaternion<T>(q_B_M.w(), q_B_M.x(), q_B_M.y(), q_B_M.z());
    changed_ = true;
  }

  // Target position anchor (world). The quaternion argument is kept for
  // call-site compatibility but is unused by the velocity+gimbal model.
  void setFocus(const Eigen::Vector3d& focus_p,
                const Eigen::Quaterniond& /*focus_q*/ = Eigen::Quaterniond::Identity()) {
    focus_p_ = Eigen::Matrix<T, 3, 1>(focus_p[0], focus_p[1], focus_p[2]);
    changed_ = true;
  }

  void setReferencePoint(const Eigen::Vector3d& point) {
    polynomial_trajectory_.clear();
    for (int i = 0; i < kSamples + 1; ++i) {
      PolynomialParams<T> poly3d;
      poly3d.poly_x = Eigen::Matrix<T, 4, 1>(0.0, 0.0, 0.0, point.x());
      poly3d.poly_y = Eigen::Matrix<T, 4, 1>(0.0, 0.0, 0.0, point.y());
      poly3d.poly_z = Eigen::Matrix<T, 4, 1>(0.0, 0.0, 0.0, point.z());
      poly3d.distance_start = 0.0;
      polynomial_trajectory_.push_back(poly3d);
    }

    changed_ = true;
  }

  void setReferenceTrajectory(const vision_common::PolynomialTrajectory& poly) {
    polynomial_trajectory_.clear();
    for (int i = 0; i < kSamples + 1; ++i) {
      PolynomialParams<T> poly3d;
      poly3d.poly_x = Eigen::Matrix<T, 4, 1>(poly.trajectory_x.a, poly.trajectory_x.b, poly.trajectory_x.c, poly.trajectory_x.y);
      poly3d.poly_y = Eigen::Matrix<T, 4, 1>(poly.trajectory_y.a, poly.trajectory_y.b, poly.trajectory_y.c, poly.trajectory_y.y);
      poly3d.poly_z = Eigen::Matrix<T, 4, 1>(poly.trajectory_z.a, poly.trajectory_z.b, poly.trajectory_z.c, poly.trajectory_z.y);
      poly3d.distance_start = static_cast<T>(poly.trajectory_x.x_s);

      polynomial_trajectory_.push_back(poly3d);
    }

    changed_ = true;
  }

  void setReferencePoint(const int node_index,
                         const Eigen::Vector3d& point) {
    if (node_index > kSamples) {
      return;
    }

    if (polynomial_trajectory_.size() != kSamples + 1) {
      polynomial_trajectory_.resize(kSamples + 1);
    }

    PolynomialParams<T> poly3d;
    poly3d.poly_x = Eigen::Matrix<T, 4, 1>(0.0, 0.0, 0.0, point.x());
    poly3d.poly_y = Eigen::Matrix<T, 4, 1>(0.0, 0.0, 0.0, point.y());
    poly3d.poly_z = Eigen::Matrix<T, 4, 1>(0.0, 0.0, 0.0, point.z());
    poly3d.distance_start = 0.0;

    polynomial_trajectory_[node_index] = poly3d;
    changed_ = true;
  }

  void setReferenceTrajectory(const int node_index,
                              const vision_common::PolynomialTrajectory& poly) {
    if (node_index > kSamples) {
      return;
    }

    if (polynomial_trajectory_.size() != kSamples + 1) {
      polynomial_trajectory_.resize(kSamples + 1);
    }

    PolynomialParams<T> poly3d;
    poly3d.poly_x = Eigen::Matrix<T, 4, 1>(poly.trajectory_x.a, poly.trajectory_x.b, poly.trajectory_x.c, poly.trajectory_x.y);
    poly3d.poly_y = Eigen::Matrix<T, 4, 1>(poly.trajectory_y.a, poly.trajectory_y.b, poly.trajectory_y.c, poly.trajectory_y.y);
    poly3d.poly_z = Eigen::Matrix<T, 4, 1>(poly.trajectory_z.a, poly.trajectory_z.b, poly.trajectory_z.c, poly.trajectory_z.y);
    poly3d.distance_start = static_cast<T>(poly.trajectory_x.x_s);

    polynomial_trajectory_[node_index] = poly3d;
    changed_ = true;
  }

  bool loadParameters(ros::NodeHandle& pnh) {
    ROS_INFO("-----------Loading MPC_PARAMS Parameters-----------");
#define GET_PARAM(name) \
  if (!vision_common::getParam(#name, name, pnh)) return false

#define GET_PARAM_(name) \
  if (!vision_common::getParam(#name, name##_, pnh)) return false

    // State costs.
    GET_PARAM_(Q_pos_xy);
    GET_PARAM_(Q_pos_z);
    GET_PARAM_(Q_yaw);
    GET_PARAM_(Q_vel);
    GET_PARAM_(Q_img_u);
    GET_PARAM_(Q_img_v);
    GET_PARAM_(Q_err);
    GET_PARAM_(Q_drate);
    GET_PARAM_(Q_gimbal);

    if (Q_pos_xy_ < 0.0 || Q_pos_z_ < 0.0 || Q_yaw_ < 0.0 || Q_vel_ < 0.0 ||
        Q_gimbal_ < 0.0) {
      ROS_ERROR("MPC: State cost Q has negative entries!");
      return false;
    }

    // Input costs.
    GET_PARAM_(R_vel);
    GET_PARAM_(R_yaw_rate);
    GET_PARAM_(R_gimbal_rate);
    GET_PARAM_(R_sv1);
    GET_PARAM_(R_sv2);

    if (R_vel_ <= 0.0 || R_yaw_rate_ <= 0.0 || R_gimbal_rate_ <= 0.0) {
      ROS_ERROR("MPC: Input cost R has non-positive entries!");
      return false;
    }

    // Set state and input cost matrices.
    Q_ = (Eigen::Matrix<T, kCostSize, 1>() <<
          Q_pos_xy_, Q_pos_xy_, Q_pos_z_,
          Q_yaw_,
          Q_vel_, Q_vel_, Q_vel_,
          Q_img_u_, Q_img_v_,
          Q_err_, Q_err_, Q_err_,
          Q_drate_,
          Q_gimbal_).finished().asDiagonal();
    R_ = (Eigen::Matrix<T, kInputSize, 1>() <<
          R_vel_, R_vel_, R_vel_,
          R_yaw_rate_, R_gimbal_rate_,
          R_sv1_, R_sv2_).finished().asDiagonal();

    // Read cost scaling values
    vision_common::getParam("state_cost_exponential", state_cost_exponential_,
                            (T)0.0, pnh);
    vision_common::getParam("input_cost_exponential", input_cost_exponential_,
                            (T)0.0, pnh);

    // Read input / state limits.
    GET_PARAM_(max_velocity);
    GET_PARAM_(max_yaw_rate);
    GET_PARAM_(max_gimbal_rate);
    GET_PARAM_(gimbal_angle_min);
    GET_PARAM_(gimbal_angle_max);

    if (max_velocity_ <= 0.0 || max_yaw_rate_ <= 0.0 || max_gimbal_rate_ <= 0.0) {
      ROS_ERROR("MPC: All rate/velocity limits must be positive non-zero values!");
      return false;
    }
    if (gimbal_angle_min_ >= gimbal_angle_max_) {
      ROS_ERROR("MPC: gimbal_angle_min must be smaller than gimbal_angle_max!");
      return false;
    }

    // Camera mount extrinsics (body -> gimbal base).
    std::vector<T> t_B_C(3), q_B_M(4);
    if (!pnh.getParam("t_B_C", t_B_C)) {
      ROS_WARN("MPC: Camera mount translation (t_B_C) is not set.");
    } else {
      t_B_C_ = Eigen::Matrix<T, 3, 1>(t_B_C[0], t_B_C[1], t_B_C[2]);
    }
    // Accept either q_B_M (preferred) or the legacy q_B_C key.
    if (pnh.getParam("q_B_M", q_B_M) || pnh.getParam("q_B_C", q_B_M)) {
      q_B_M_ = Eigen::Quaternion<T>(q_B_M[0], q_B_M[1], q_B_M[2], q_B_M[3]);
    } else {
      ROS_WARN("MPC: Camera mount rotation (q_B_M / q_B_C) is not set.");
    }

    vision_common::getParam("print_info", print_info_, false, pnh);
    if (print_info_) ROS_INFO("MPC: Informative printing enabled.");

    changed_ = true;

#undef GET_PARAM
#undef GET_PARAM_

    return true;
  }

  T state_cost_exponential_;
  T input_cost_exponential_;

  // Input / state limits.
  T max_velocity_;       // max commanded speed per axis [m/s]
  T max_yaw_rate_;       // max yaw rate (camera pan) [rad/s]
  T max_gimbal_rate_;    // max gimbal pitch rate (camera tilt) [rad/s]
  T gimbal_angle_min_;   // gimbal travel lower bound [rad]
  T gimbal_angle_max_;   // gimbal travel upper bound [rad]

  Eigen::Matrix<T, 3, 1> t_B_C_;
  Eigen::Quaternion<T> q_B_M_;

  Eigen::Matrix<T, 3, 1> focus_p_;

  std::vector<PolynomialParams<T>> polynomial_trajectory_;

  Eigen::Matrix<T, kCostSize, kCostSize> Q_;
  Eigen::Matrix<T, kInputSize, kInputSize> R_;

  T Q_pos_xy_;
  T Q_pos_z_;
  T Q_yaw_;
  T Q_vel_;
  T Q_img_u_;
  T Q_img_v_;
  T Q_err_;
  T Q_drate_;
  T Q_gimbal_;

  T R_vel_;
  T R_yaw_rate_;
  T R_gimbal_rate_;
  T R_sv1_;
  T R_sv2_;

  std::vector<T> sv1_scalings_;
  std::vector<T> sv2_scalings_;

  bool changed_;
  bool print_info_;
};
}  // namespace mpc
