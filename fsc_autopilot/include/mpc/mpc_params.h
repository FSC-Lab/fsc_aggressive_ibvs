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

template <typename T>
class MpcParams {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  MpcParams()
      : changed_(false),
        print_info_(false),
        state_cost_exponential_(0.0),
        input_cost_exponential_(0.0),
        max_thrust_rate_(0.0),
        max_bodyrate_x_(0.0),
        max_bodyrate_y_(0.0),
        max_bodyrate_z_(0.0),
        min_thrust_(0.0),
        max_thrust_(0.0),
        t_B_C_(Eigen::Matrix<T, 3, 1>::Zero()),
        q_B_C_(Eigen::Quaternion<T>(1.0, 0.0, 0.0, 0.0)),
        focus_p_(Eigen::Matrix<T, 3, 1>::Zero()),
        focus_q_(Eigen::Quaternion<T>(1.0, 0.0, 0.0, 0.0)),
        gate_p_(Eigen::Matrix<T, 3, 1>::Zero()),
        gate_q_(Eigen::Quaternion<T>(1.0, 0.0, 0.0, 0.0)), 
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

  void setVelocityWeight(const double Q_vel) {
    Q_.block((int)ReferenceIndex::VEL_X, (int)ReferenceIndex::VEL_X, 3, 3) = (Eigen::Matrix<T, 3, 1>()
        << Q_vel, Q_vel, Q_vel).finished().asDiagonal();

    Q_vel_ = Q_vel;

    changed_ = true;
  }

  void setVisualServoWeight(const double Q_err) {
    Q_.block((int)ReferenceIndex::ERR_X, (int)ReferenceIndex::ERR_X, 3, 3) = (Eigen::Matrix<T, 3, 1>()
        << Q_err, Q_err, Q_err).finished().asDiagonal();

    Q_err_ = Q_err_;

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

  void setCameraExtrinsics(const Eigen::Vector3d& t_B_C,
                           const Eigen::Quaterniond& q_B_C) {
    t_B_C_ = Eigen::Matrix<T, 3, 1>(t_B_C[0], t_B_C[1], t_B_C[2]);
    q_B_C_ = Eigen::Quaternion<T>(q_B_C.w(), q_B_C.x(), q_B_C.y(), q_B_C.z());
    changed_ = true;
  }

  void setFocus(const Eigen::Vector3d& focus_p,
                const Eigen::Quaterniond& focus_q) {
    focus_p_ = Eigen::Matrix<T, 3, 1>(focus_p[0], focus_p[1], focus_p[2]);                 
    focus_q_ = Eigen::Quaternion<T>(focus_q.w(), focus_q.x(), focus_q.y(), focus_q.z());
    changed_ = true;
  }

  void setGate(const Eigen::Vector3d& gate_p,
               const Eigen::Quaterniond& gate_q) {
    gate_p_ = Eigen::Matrix<T, 3, 1>(gate_p[0], gate_p[1], gate_p[2]);                 
    gate_q_ = Eigen::Quaternion<T>(gate_q.w(), gate_q.x(), gate_q.y(), gate_q.z());
    changed_ = true;
  }

  void setSize(const double gate_width,
               const double gate_height,
               const double quad_radius) {
    gate_width_ = (T)gate_width;
    gate_height_ = (T)gate_height;
    quad_radius_ = (T)quad_radius;
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

    GET_PARAM_(Q_pos_xy);
    GET_PARAM_(Q_pos_z);
    GET_PARAM_(Q_att);
    GET_PARAM_(Q_vel);
    GET_PARAM_(Q_thr);
    GET_PARAM_(Q_img_u);
    GET_PARAM_(Q_img_v);
    GET_PARAM_(Q_err);
    GET_PARAM_(Q_drate);

    if (Q_pos_xy_ < 0.0 || Q_pos_z_ < 0.0 || Q_att_ < 0.0 || Q_vel_ < 0.0) {
      ROS_ERROR("MPC: State cost Q has negative enries!");
      return false;
    }

    if (Q_thr_ < 0.0) {
      ROS_ERROR("MPC: State cost Q has negative enries!");
      return false;
    }

    // Read input costs.
    GET_PARAM_(R_thrust);
    GET_PARAM_(R_roll);
    GET_PARAM_(R_pitch);
    GET_PARAM_(R_yaw);
    GET_PARAM_(R_sv1);
    GET_PARAM_(R_sv2);

    // Check whether all input costs are positive.
    if (R_thrust_ <= 0.0 || R_roll_ <= 0.0 || R_pitch_ <= 0.0 || R_yaw_ <= 0.0) {
      ROS_ERROR("MPC: Input cost R has negative enries!");
      return false;
    }

    // Set state and input cost matrices.
    Q_ = (Eigen::Matrix<T, kCostSize, 1>() <<
          Q_pos_xy_, Q_pos_xy_, Q_pos_z_,
          Q_att_, Q_att_, Q_att_, Q_att_,
          Q_vel_, Q_vel_, Q_vel_, Q_thr_,
          Q_img_u_, Q_img_v_,
          Q_err_,  Q_err_, Q_err_, Q_drate_).finished().asDiagonal();
    R_ = (Eigen::Matrix<T, kInputSize, 1>() <<
          R_thrust_, R_roll_, R_pitch_, R_yaw_,
          R_sv1_, R_sv2_).finished().asDiagonal();

    // Read cost scaling values
    vision_common::getParam("state_cost_exponential", state_cost_exponential_,
                            (T)0.0, pnh);
    vision_common::getParam("input_cost_exponential", input_cost_exponential_,
                            (T)0.0, pnh);

    // Read input limits.
    GET_PARAM_(max_thrust_rate);
    GET_PARAM_(max_bodyrate_x);
    GET_PARAM_(max_bodyrate_y);
    GET_PARAM_(max_bodyrate_z);
    GET_PARAM_(min_thrust);
    GET_PARAM_(max_thrust);
    GET_PARAM_(gate_width);
    GET_PARAM_(gate_height);
    GET_PARAM_(quad_radius);

    // Check whether all input limits are positive.
    if (max_thrust_rate_ <= 0.0 || max_bodyrate_x_ <= 0.0 ||
        max_bodyrate_y_ <= 0.0 || max_bodyrate_z_ <= 0.0 ||
        min_thrust_ <= 0.0 || max_thrust_ <= 0.0) {
      ROS_ERROR("MPC: All limits must be positive non-zero values!");
      return false;
    }

    // Optional parameters
    std::vector<T> t_B_C(3), q_B_C(4);
    if (!pnh.getParam("t_B_C", t_B_C)) {
      ROS_WARN("MPC: Camera extrinsic translation is not set.");
    } else {
      t_B_C_ = Eigen::Matrix<T, 3, 1>(t_B_C[0], t_B_C[1], t_B_C[2]);
    }
    if (!pnh.getParam("q_B_C", q_B_C)) {
      ROS_WARN("MPC: Camera extrinsic rotation is not set.");
    } else {
      q_B_C_ = Eigen::Quaternion<T>(q_B_C[0], q_B_C[1], q_B_C[2], q_B_C[3]);
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

  T max_thrust_rate_;
  T max_bodyrate_x_;
  T max_bodyrate_y_;
  T max_bodyrate_z_;

  T min_thrust_;
  T max_thrust_;

  T gate_width_;
  T gate_height_;
  T quad_radius_;

  Eigen::Matrix<T, 3, 1> t_B_C_;
  Eigen::Quaternion<T> q_B_C_;

  Eigen::Matrix<T, 3, 1> focus_p_;
  Eigen::Quaternion<T> focus_q_; 

  Eigen::Matrix<T, 3, 1> gate_p_;
  Eigen::Quaternion<T> gate_q_; 

  std::vector<PolynomialParams<T>> polynomial_trajectory_;

  Eigen::Matrix<T, kCostSize, kCostSize> Q_;
  Eigen::Matrix<T, kInputSize, kInputSize> R_;

  T Q_pos_xy_;
  T Q_pos_z_;
  T Q_att_;
  T Q_vel_;
  T Q_thr_;
  T Q_img_u_;
  T Q_img_v_;
  T Q_err_; 
  T Q_drate_;

  T R_thrust_;
  T R_roll_;
  T R_pitch_;
  T R_yaw_;
  T R_sv1_;
  T R_sv2_;

  std::vector<T> sv1_scalings_;
  std::vector<T> sv2_scalings_;

  bool changed_;
  bool print_info_;
};
}  // namespace mpc