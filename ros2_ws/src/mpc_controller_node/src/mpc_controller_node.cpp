// mpc_controller_node
//
// Thin rclcpp wrapper around the ROS-free mpc_core library. It converts ROS2
// messages to/from the mpc_core POD types, runs the velocity-body + pitch-gimbal
// IBVS NMPC on a fixed-rate timer, and publishes a fsc_vision_msgs/ControlCommand
// (consumed by px4_offboard_bridge) plus a predicted-path visualization.
//
// This node is PX4-agnostic: it works in world ENU (ROS convention). Frame
// conversion to PX4 NED lives in px4_offboard_bridge.

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Eigen>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/float64.hpp>

#include "fsc_vision_msgs/msg/control_command.hpp"
#include "fsc_vision_msgs/msg/target_bearing.hpp"

#include "mpc_core/mpc_controller.h"
#include "mpc_core/mpc_params.h"
#include "mpc_core/types.h"

namespace {

double stampToSec(const builtin_interfaces::msg::Time& t) {
  return static_cast<double>(t.sec) + static_cast<double>(t.nanosec) * 1e-9;
}

builtin_interfaces::msg::Time secToStamp(double s) {
  builtin_interfaces::msg::Time t;
  t.sec = static_cast<int32_t>(s);
  t.nanosec = static_cast<uint32_t>((s - static_cast<double>(t.sec)) * 1e9);
  return t;
}

}  // namespace

class MpcControllerNode : public rclcpp::Node {
 public:
  MpcControllerNode() : rclcpp::Node("mpc_controller_node") {
    declareParameters();
    loadParameters();

    if (!params_.configure(cfg_)) {
      RCLCPP_FATAL(get_logger(), "Failed to configure MPC parameters.");
      throw std::runtime_error("MPC parameter configuration failed");
    }

    // Static target anchor (world). Updated live by target_point subscription.
    params_.setFocus(target_position_);
    params_.setReferencePoint(target_position_);

    controller_.initialize(params_);

    // I/O.
    const rclcpp::QoS sensor_qos = rclcpp::SensorDataQoS();

    pub_command_ = create_publisher<fsc_vision_msgs::msg::ControlCommand>(
        "mpc/control_command", 10);
    pub_path_ = create_publisher<nav_msgs::msg::Path>(
        "mpc/predicted_path", 1);

    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
        "odometry", sensor_qos,
        std::bind(&MpcControllerNode::odomCallback, this, std::placeholders::_1));
    sub_bearing_ = create_subscription<fsc_vision_msgs::msg::TargetBearing>(
        "target_bearing", sensor_qos,
        std::bind(&MpcControllerNode::bearingCallback, this, std::placeholders::_1));
    sub_target_point_ = create_subscription<geometry_msgs::msg::PointStamped>(
        "target_point", 10,
        std::bind(&MpcControllerNode::targetPointCallback, this, std::placeholders::_1));
    sub_reference_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "reference_pose", 10,
        std::bind(&MpcControllerNode::referenceCallback, this, std::placeholders::_1));
    sub_gimbal_fb_ = create_subscription<std_msgs::msg::Float64>(
        "gimbal_feedback", 10,
        std::bind(&MpcControllerNode::gimbalFeedbackCallback, this, std::placeholders::_1));

    const auto period = std::chrono::duration<double>(1.0 / control_rate_);
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&MpcControllerNode::controlStep, this));

    RCLCPP_INFO(get_logger(), "mpc_controller_node up at %.1f Hz.", control_rate_);
  }

 private:
  // ---- parameter handling -------------------------------------------------
  void declareParameters() {
    declare_parameter("control_rate", 20.0);

    declare_parameter("Q_pos_xy", 200.0);
    declare_parameter("Q_pos_z", 200.0);
    declare_parameter("Q_yaw", 50.0);
    declare_parameter("Q_vel", 10.0);
    declare_parameter("Q_img_u", 0.0);
    declare_parameter("Q_img_v", 0.0);
    declare_parameter("Q_err", 0.0);
    declare_parameter("Q_drate", 0.0);
    declare_parameter("Q_gimbal", 1.0);

    declare_parameter("R_vel", 1.0);
    declare_parameter("R_yaw_rate", 20.0);
    declare_parameter("R_gimbal_rate", 10.0);
    declare_parameter("R_sv1", 0.001);
    declare_parameter("R_sv2", 0.001);

    declare_parameter("state_cost_exponential", 0.0);
    declare_parameter("input_cost_exponential", 0.0);

    declare_parameter("max_velocity", 8.0);
    declare_parameter("max_yaw_rate", 1.5);
    declare_parameter("max_gimbal_rate", 3.0);
    declare_parameter("gimbal_angle_min", -1.5708);
    declare_parameter("gimbal_angle_max", 0.5236);

    declare_parameter("t_B_C", std::vector<double>{0.0, 0.0, 0.15});
    declare_parameter("q_B_M", std::vector<double>{0.5, -0.5, 0.5, -0.5});  // w,x,y,z

    declare_parameter("target_position", std::vector<double>{5.0, 0.0, 1.0});
    declare_parameter("reference_position", std::vector<double>{0.0, 0.0, 3.0});
    declare_parameter("reference_heading", 0.0);

    declare_parameter("print_info", false);
    declare_parameter("world_frame", std::string("odom"));
  }

  void loadParameters() {
    control_rate_ = get_parameter("control_rate").as_double();

    cfg_.Q_pos_xy = get_parameter("Q_pos_xy").as_double();
    cfg_.Q_pos_z = get_parameter("Q_pos_z").as_double();
    cfg_.Q_yaw = get_parameter("Q_yaw").as_double();
    cfg_.Q_vel = get_parameter("Q_vel").as_double();
    cfg_.Q_img_u = get_parameter("Q_img_u").as_double();
    cfg_.Q_img_v = get_parameter("Q_img_v").as_double();
    cfg_.Q_err = get_parameter("Q_err").as_double();
    cfg_.Q_drate = get_parameter("Q_drate").as_double();
    cfg_.Q_gimbal = get_parameter("Q_gimbal").as_double();

    cfg_.R_vel = get_parameter("R_vel").as_double();
    cfg_.R_yaw_rate = get_parameter("R_yaw_rate").as_double();
    cfg_.R_gimbal_rate = get_parameter("R_gimbal_rate").as_double();
    cfg_.R_sv1 = get_parameter("R_sv1").as_double();
    cfg_.R_sv2 = get_parameter("R_sv2").as_double();

    cfg_.state_cost_exponential = get_parameter("state_cost_exponential").as_double();
    cfg_.input_cost_exponential = get_parameter("input_cost_exponential").as_double();

    cfg_.max_velocity = get_parameter("max_velocity").as_double();
    cfg_.max_yaw_rate = get_parameter("max_yaw_rate").as_double();
    cfg_.max_gimbal_rate = get_parameter("max_gimbal_rate").as_double();
    cfg_.gimbal_angle_min = get_parameter("gimbal_angle_min").as_double();
    cfg_.gimbal_angle_max = get_parameter("gimbal_angle_max").as_double();

    const auto t = get_parameter("t_B_C").as_double_array();
    if (t.size() == 3) cfg_.t_B_C = Eigen::Vector3d(t[0], t[1], t[2]);
    const auto q = get_parameter("q_B_M").as_double_array();
    if (q.size() == 4) cfg_.q_B_M = Eigen::Quaterniond(q[0], q[1], q[2], q[3]);  // w,x,y,z

    const auto tp = get_parameter("target_position").as_double_array();
    if (tp.size() == 3) target_position_ = Eigen::Vector3d(tp[0], tp[1], tp[2]);
    const auto rp = get_parameter("reference_position").as_double_array();
    if (rp.size() == 3) reference_position_ = Eigen::Vector3d(rp[0], rp[1], rp[2]);
    reference_heading_ = get_parameter("reference_heading").as_double();

    cfg_.print_info = get_parameter("print_info").as_bool();
    world_frame_ = get_parameter("world_frame").as_string();
  }

  // ---- subscriptions ------------------------------------------------------
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(mutex_);
    latest_state_.timestamp = stampToSec(msg->header.stamp);
    latest_state_.position = Eigen::Vector3d(msg->pose.pose.position.x,
                                             msg->pose.pose.position.y,
                                             msg->pose.pose.position.z);
    latest_state_.orientation = Eigen::Quaterniond(msg->pose.pose.orientation.w,
                                                   msg->pose.pose.orientation.x,
                                                   msg->pose.pose.orientation.y,
                                                   msg->pose.pose.orientation.z);
    // Odometry twist is body-frame in ROS convention; rotate to world (ENU).
    const Eigen::Vector3d v_body(msg->twist.twist.linear.x,
                                 msg->twist.twist.linear.y,
                                 msg->twist.twist.linear.z);
    latest_state_.velocity = latest_state_.orientation * v_body;
    have_odom_ = true;
  }

  void bearingCallback(const fsc_vision_msgs::msg::TargetBearing::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(mutex_);
    latest_state_.bearing = Eigen::Quaterniond(msg->bearing.w, msg->bearing.x,
                                               msg->bearing.y, msg->bearing.z);
    latest_state_.distance = msg->range;
    have_bearing_ = msg->valid;
  }

  void targetPointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(mutex_);
    target_position_ = Eigen::Vector3d(msg->point.x, msg->point.y, msg->point.z);
    params_.setFocus(target_position_);
    params_.setReferencePoint(target_position_);  // marks params changed_
  }

  void referenceCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(mutex_);
    reference_position_ = Eigen::Vector3d(msg->pose.position.x,
                                          msg->pose.position.y,
                                          msg->pose.position.z);
    const Eigen::Quaterniond q(msg->pose.orientation.w, msg->pose.orientation.x,
                               msg->pose.orientation.y, msg->pose.orientation.z);
    reference_heading_ = mpc_core::yawFromQuaternion(q);
  }

  void gimbalFeedbackCallback(const std_msgs::msg::Float64::SharedPtr msg) {
    controller_.setGimbalAngle(msg->data);
  }

  // ---- control loop -------------------------------------------------------
  void controlStep() {
    mpc_core::StateEstimate state;
    mpc_core::Trajectory reference;
    {
      std::lock_guard<std::mutex> lk(mutex_);
      if (!have_odom_) {
        return;  // wait for the first odometry
      }
      state = latest_state_;
      // If the target is not currently visible, keep the last bearing but the
      // higher-level supervisor (not this node) is responsible for what to do;
      // here we simply keep tracking the last known line of sight.
      reference = buildReference(state);
    }

    std::list<mpc_core::ControlCommand> command_queue;
    std::vector<mpc_core::StateEstimate> predicted;
    const bool ok = controller_.run(state, reference, params_,
                                    command_queue, predicted);
    if (!ok || command_queue.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "MPC solve failed; holding.");
      return;
    }

    publishCommand(command_queue.front());
    publishPath(predicted);
  }

  // Desired drone reference: hold the reference standoff position/heading. The
  // IBVS costs (img/err) keep the target framed; position cost keeps the drone
  // near the standoff. Single-point trajectory (receding-horizon hold).
  mpc_core::Trajectory buildReference(const mpc_core::StateEstimate& /*state*/) {
    mpc_core::Trajectory traj;
    mpc_core::TrajectoryPoint pt;
    pt.time_from_start = 0.0;
    pt.position = reference_position_;
    pt.orientation = Eigen::Quaterniond(
        Eigen::AngleAxisd(reference_heading_, Eigen::Vector3d::UnitZ()));
    pt.velocity = Eigen::Vector3d::Zero();
    pt.distance_rate = 0.0;
    traj.points.push_back(pt);
    return traj;
  }

  void publishCommand(const mpc_core::ControlCommand& c) {
    fsc_vision_msgs::msg::ControlCommand msg;
    msg.header.stamp = now();
    msg.header.frame_id = world_frame_;
    msg.control_mode = fsc_vision_msgs::msg::ControlCommand::VELOCITY;
    msg.armed = c.armed;
    msg.expected_execution_time = secToStamp(c.expected_execution_time);
    msg.velocity.x = c.velocity.x();
    msg.velocity.y = c.velocity.y();
    msg.velocity.z = c.velocity.z();
    msg.yaw_rate = c.yaw_rate;
    msg.gimbal_angle = c.gimbal_angle;
    msg.slack_variables.assign(c.slack_variables.begin(),
                               c.slack_variables.end());
    pub_command_->publish(msg);
  }

  void publishPath(const std::vector<mpc_core::StateEstimate>& predicted) {
    if (pub_path_->get_subscription_count() == 0) return;
    nav_msgs::msg::Path path;
    path.header.stamp = now();
    path.header.frame_id = world_frame_;
    for (const auto& s : predicted) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = path.header;
      ps.pose.position.x = s.position.x();
      ps.pose.position.y = s.position.y();
      ps.pose.position.z = s.position.z();
      ps.pose.orientation.w = s.orientation.w();
      ps.pose.orientation.x = s.orientation.x();
      ps.pose.orientation.y = s.orientation.y();
      ps.pose.orientation.z = s.orientation.z();
      path.poses.push_back(ps);
    }
    pub_path_->publish(path);
  }

  // ---- members ------------------------------------------------------------
  mpc::MpcController<double> controller_;
  mpc::MpcParams<double> params_;
  mpc::MpcParamsConfig cfg_;

  std::mutex mutex_;
  mpc_core::StateEstimate latest_state_;
  bool have_odom_{false};
  bool have_bearing_{false};

  Eigen::Vector3d target_position_{5.0, 0.0, 1.0};
  Eigen::Vector3d reference_position_{0.0, 0.0, 3.0};
  double reference_heading_{0.0};

  double control_rate_{20.0};
  std::string world_frame_{"odom"};

  rclcpp::Publisher<fsc_vision_msgs::msg::ControlCommand>::SharedPtr pub_command_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<fsc_vision_msgs::msg::TargetBearing>::SharedPtr sub_bearing_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_target_point_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_reference_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_gimbal_fb_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MpcControllerNode>());
  rclcpp::shutdown();
  return 0;
}
