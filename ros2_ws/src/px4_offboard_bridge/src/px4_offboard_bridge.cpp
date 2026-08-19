// px4_offboard_bridge
//
// Bridges the PX4-agnostic mpc_controller_node to a PX4 autopilot over the
// uXRCE-DDS bus. Responsibilities, all isolated here so the MPC stack stays
// PX4-free:
//
//   1. Frame conversion  ROS (world ENU / body FLU)  <->  PX4 (world NED / FRD).
//   2. Consume fsc_vision_msgs/ControlCommand (VELOCITY mode: world-ENU linear
//      velocity + yaw_rate + gimbal_angle) and stream PX4 OffboardControlMode +
//      TrajectorySetpoint (NED velocity + yawspeed).
//   3. Offboard-engage + arm/disarm state machine via VehicleCommand.
//   4. Republish PX4 VehicleOdometry as nav_msgs/Odometry (ENU) for the node.
//   5. Republish the gimbal angle as std_msgs/Float64 for the gz gimbal joint.
//
// PX4 offboard requires OffboardControlMode + a setpoint to be streamed at >2 Hz
// before and during offboard, so the heartbeat runs on a fixed-rate timer that
// re-publishes the most recent commanded setpoint.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <string>

#include <Eigen/Eigen>
#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

#include "fsc_vision_msgs/msg/control_command.hpp"

#include "px4_offboard_bridge/frame_transforms.h"

namespace {
constexpr float kNaNf = std::numeric_limits<float>::quiet_NaN();
}  // namespace

class Px4OffboardBridge : public rclcpp::Node {
 public:
  Px4OffboardBridge() : rclcpp::Node("px4_offboard_bridge") {
    declare_parameter("heartbeat_rate", 50.0);
    declare_parameter("command_timeout", 0.5);   // [s] stale-command watchdog
    declare_parameter("enable_auto_arm", true);
    declare_parameter("world_frame", std::string("odom"));
    declare_parameter("body_frame", std::string("base_link"));

    heartbeat_rate_ = get_parameter("heartbeat_rate").as_double();
    command_timeout_ = get_parameter("command_timeout").as_double();
    enable_auto_arm_ = get_parameter("enable_auto_arm").as_bool();
    world_frame_ = get_parameter("world_frame").as_string();
    body_frame_ = get_parameter("body_frame").as_string();

    // PX4 uXRCE-DDS publishes best-effort; match it on the /fmu/out topics.
    rclcpp::QoS px4_qos(rclcpp::KeepLast(5));
    px4_qos.best_effort();
    px4_qos.durability_volatile();

    pub_offboard_mode_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
        "fmu/in/offboard_control_mode", 10);
    pub_trajectory_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        "fmu/in/trajectory_setpoint", 10);
    pub_vehicle_command_ = create_publisher<px4_msgs::msg::VehicleCommand>(
        "fmu/in/vehicle_command", 10);

    pub_odom_ = create_publisher<nav_msgs::msg::Odometry>("odometry", 10);
    pub_gimbal_ = create_publisher<std_msgs::msg::Float64>("gimbal_cmd", 10);

    sub_command_ = create_subscription<fsc_vision_msgs::msg::ControlCommand>(
        "mpc/control_command", 10,
        std::bind(&Px4OffboardBridge::commandCallback, this, std::placeholders::_1));
    sub_px4_odom_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
        "fmu/out/vehicle_odometry", px4_qos,
        std::bind(&Px4OffboardBridge::odomCallback, this, std::placeholders::_1));
    sub_px4_status_ = create_subscription<px4_msgs::msg::VehicleStatus>(
        "fmu/out/vehicle_status", px4_qos,
        std::bind(&Px4OffboardBridge::statusCallback, this, std::placeholders::_1));

    const auto period = std::chrono::duration<double>(1.0 / heartbeat_rate_);
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&Px4OffboardBridge::heartbeat, this));

    RCLCPP_INFO(get_logger(),
                "px4_offboard_bridge up (heartbeat %.1f Hz, auto_arm=%s).",
                heartbeat_rate_, enable_auto_arm_ ? "true" : "false");
  }

 private:
  uint64_t nowUs() const {
    return static_cast<uint64_t>(get_clock()->now().nanoseconds() / 1000);
  }

  // ---- inbound: MPC control command (world ENU) ---------------------------
  void commandCallback(const fsc_vision_msgs::msg::ControlCommand::SharedPtr msg) {
    if (msg->control_mode != fsc_vision_msgs::msg::ControlCommand::VELOCITY) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Ignoring control_mode %u (only VELOCITY supported).",
                           msg->control_mode);
      return;
    }

    const Eigen::Vector3d v_enu(msg->velocity.x, msg->velocity.y, msg->velocity.z);
    {
      std::lock_guard<std::mutex> lk(mutex_);
      vel_ned_ = px4_bridge::swapEnuNed(v_enu);
      yawspeed_ned_ = px4_bridge::yawRateEnuToNed(msg->yaw_rate);
      armed_request_ = msg->armed;
      last_command_time_ = get_clock()->now();
      have_command_ = true;
    }

    // Forward the gimbal tilt straight to the gz joint (radians).
    std_msgs::msg::Float64 g;
    g.data = msg->gimbal_angle;
    pub_gimbal_->publish(g);
  }

  // ---- inbound: PX4 odometry (NED/FRD) -> ROS Odometry (ENU/FLU) ----------
  void odomCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
    const Eigen::Vector3d p_ned(msg->position[0], msg->position[1], msg->position[2]);
    const Eigen::Quaterniond q_ned(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);
    const Eigen::Vector3d v_ned(msg->velocity[0], msg->velocity[1], msg->velocity[2]);
    const Eigen::Vector3d w_frd(msg->angular_velocity[0], msg->angular_velocity[1],
                                msg->angular_velocity[2]);

    const Eigen::Vector3d p_enu = px4_bridge::swapEnuNed(p_ned);
    const Eigen::Quaterniond q_enu = px4_bridge::attitudeNedToEnu(q_ned);
    const Eigen::Vector3d v_enu = px4_bridge::swapEnuNed(v_ned);
    // nav_msgs/Odometry twist is expressed in the child (body) frame; the node
    // rotates it back to world by the orientation. Give it body-FLU velocity.
    const Eigen::Vector3d v_body_flu = q_enu.inverse() * v_enu;
    const Eigen::Vector3d w_flu = px4_bridge::swapFrdFlu(w_frd);

    nav_msgs::msg::Odometry out;
    out.header.stamp = now();
    out.header.frame_id = world_frame_;
    out.child_frame_id = body_frame_;
    out.pose.pose.position.x = p_enu.x();
    out.pose.pose.position.y = p_enu.y();
    out.pose.pose.position.z = p_enu.z();
    out.pose.pose.orientation.w = q_enu.w();
    out.pose.pose.orientation.x = q_enu.x();
    out.pose.pose.orientation.y = q_enu.y();
    out.pose.pose.orientation.z = q_enu.z();
    out.twist.twist.linear.x = v_body_flu.x();
    out.twist.twist.linear.y = v_body_flu.y();
    out.twist.twist.linear.z = v_body_flu.z();
    out.twist.twist.angular.x = w_flu.x();
    out.twist.twist.angular.y = w_flu.y();
    out.twist.twist.angular.z = w_flu.z();
    pub_odom_->publish(out);
  }

  void statusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(mutex_);
    arming_state_ = msg->arming_state;
    nav_state_ = msg->nav_state;
  }

  // ---- outbound: offboard heartbeat + state machine -----------------------
  void heartbeat() {
    Eigen::Vector3d vel_ned;
    double yawspeed_ned;
    bool armed_request;
    bool command_fresh;
    uint8_t arming_state, nav_state;
    {
      std::lock_guard<std::mutex> lk(mutex_);
      vel_ned = vel_ned_;
      yawspeed_ned = yawspeed_ned_;
      armed_request = armed_request_;
      command_fresh = have_command_ &&
          (get_clock()->now() - last_command_time_).seconds() < command_timeout_;
      arming_state = arming_state_;
      nav_state = nav_state_;
    }

    // Zero-velocity hold if no fresh command (watchdog) — never stream stale.
    if (!command_fresh) {
      vel_ned.setZero();
      yawspeed_ned = 0.0;
    }

    publishOffboardControlMode();
    publishTrajectorySetpoint(vel_ned, yawspeed_ned);

    // After a few setpoints have been streamed, request offboard then arm.
    if (offboard_counter_ < kArmAfterSetpoints) {
      ++offboard_counter_;
      return;
    }
    if (offboard_counter_ == kArmAfterSetpoints) {
      ++offboard_counter_;  // one-shot engage
      if (armed_request && enable_auto_arm_) {
        engageOffboard();
        arm();
      }
    }

    // Honour a later disarm request from the controller.
    if (!armed_request &&
        arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED) {
      disarm();
    }
    (void)nav_state;
  }

  void publishOffboardControlMode() {
    px4_msgs::msg::OffboardControlMode m{};
    m.timestamp = nowUs();
    m.position = false;
    m.velocity = true;
    m.acceleration = false;
    m.attitude = false;
    m.body_rate = false;
    pub_offboard_mode_->publish(m);
  }

  void publishTrajectorySetpoint(const Eigen::Vector3d& vel_ned,
                                 double yawspeed_ned) {
    px4_msgs::msg::TrajectorySetpoint sp{};
    sp.timestamp = nowUs();
    sp.position = {kNaNf, kNaNf, kNaNf};
    sp.velocity = {static_cast<float>(vel_ned.x()),
                   static_cast<float>(vel_ned.y()),
                   static_cast<float>(vel_ned.z())};
    sp.acceleration = {kNaNf, kNaNf, kNaNf};
    sp.jerk = {kNaNf, kNaNf, kNaNf};
    sp.yaw = kNaNf;  // use yawspeed only
    sp.yawspeed = static_cast<float>(yawspeed_ned);
    pub_trajectory_->publish(sp);
  }

  void sendVehicleCommand(uint16_t command, float p1 = 0.0f, float p2 = 0.0f) {
    px4_msgs::msg::VehicleCommand cmd{};
    cmd.timestamp = nowUs();
    cmd.param1 = p1;
    cmd.param2 = p2;
    cmd.command = command;
    cmd.target_system = 1;
    cmd.target_component = 1;
    cmd.source_system = 1;
    cmd.source_component = 1;
    cmd.from_external = true;
    pub_vehicle_command_->publish(cmd);
  }

  void engageOffboard() {
    // VEHICLE_CMD_DO_SET_MODE: base_mode=1 (custom), main_mode=6 (offboard).
    sendVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
                       1.0f, 6.0f);
    RCLCPP_INFO(get_logger(), "Requested offboard mode.");
  }

  void arm() {
    sendVehicleCommand(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
    RCLCPP_INFO(get_logger(), "Arm command sent.");
  }

  void disarm() {
    sendVehicleCommand(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0f);
    RCLCPP_WARN(get_logger(), "Disarm command sent.");
  }

  // ---- members ------------------------------------------------------------
  static constexpr int kArmAfterSetpoints = 20;  // ~0.4 s at 50 Hz

  double heartbeat_rate_{50.0};
  double command_timeout_{0.5};
  bool enable_auto_arm_{true};
  std::string world_frame_{"odom"};
  std::string body_frame_{"base_link"};

  std::mutex mutex_;
  Eigen::Vector3d vel_ned_{Eigen::Vector3d::Zero()};
  double yawspeed_ned_{0.0};
  bool armed_request_{false};
  bool have_command_{false};
  rclcpp::Time last_command_time_{0, 0, RCL_ROS_TIME};
  uint8_t arming_state_{0};
  uint8_t nav_state_{0};
  int offboard_counter_{0};

  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr pub_offboard_mode_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr pub_trajectory_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr pub_vehicle_command_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_gimbal_;
  rclcpp::Subscription<fsc_vision_msgs::msg::ControlCommand>::SharedPtr sub_command_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr sub_px4_odom_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr sub_px4_status_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Px4OffboardBridge>());
  rclcpp::shutdown();
  return 0;
}
