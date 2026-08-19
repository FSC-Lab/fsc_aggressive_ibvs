// target_bearing_node
//
// Ground-truth line-of-sight source for the IBVS NMPC. Given the drone
// odometry (world/ENU), the target's ground-truth pose (world/ENU, from the gz
// bbox/pose plugin via ros_gz_bridge) and the current gimbal tilt, it computes
// the bearing quaternion f1 and range r1 that the model expects, and publishes
// them as fsc_vision_msgs/TargetBearing. It also republishes the target world
// position as a PointStamped so the MPC node can anchor its focus/reference.
//
// Conventions are matched *exactly* to utils/pose_track_mpc_gimbal.cpp and the
// ROS1 vision_common pipeline:
//   * gimbaled camera rotation   q_B_C = q_B_M (x) q_pitch(theta_g),
//     q_pitch = (cos(theta_g/2), 0, sin(theta_g/2), 0)   (about camera y-axis)
//   * LOS in camera frame        p_lc_c = q_B_C^-1 * ( q_W_B^-1 * (tgt - p) - t_B_C )
//   * range r1 = ||p_lc_c||
//   * bearing f1 = shortest-arc rotation from boresight +Z onto p_lc_c
//     (vision_common::NormalVector::setFromVector), i.e. LOS = R(f1) * e_z.

#include <cmath>
#include <memory>
#include <mutex>
#include <string>

#include <Eigen/Eigen>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>

#include "fsc_vision_msgs/msg/target_bearing.hpp"

namespace {

// Shortest-arc rotation taking unit vector a onto unit vector b, with a_perp as
// the fallback axis for the antiparallel (180 deg) case. Mirrors
// vision_common::getAngleAxisFromTwoNormals + angleAxisToQuaternion, which is
// how the ROS1 bearing quaternion was built.
Eigen::Quaterniond quatFromTwoNormals(const Eigen::Vector3d& a,
                                      const Eigen::Vector3d& b,
                                      const Eigen::Vector3d& a_perp) {
  const Eigen::Vector3d cross = a.cross(b);
  const double cross_norm = cross.norm();
  const double c = std::max(-1.0, std::min(1.0, a.dot(b)));
  Eigen::Vector3d rvec;
  if (cross_norm < 1e-6) {
    rvec = (c > 0.0) ? Eigen::Vector3d(Eigen::Vector3d::Zero())
                     : Eigen::Vector3d(a_perp * M_PI);
  } else {
    rvec = cross * (std::acos(c) / cross_norm);
  }
  const double angle = rvec.norm();
  if (angle < 1e-9) return Eigen::Quaterniond::Identity();
  return Eigen::Quaterniond(Eigen::AngleAxisd(angle, rvec / angle));
}

}  // namespace

class TargetBearingNode : public rclcpp::Node {
 public:
  TargetBearingNode() : rclcpp::Node("target_bearing_node") {
    declare_parameter("publish_rate", 30.0);
    declare_parameter("t_B_C", std::vector<double>{0.0, 0.0, 0.15});
    declare_parameter("q_B_M", std::vector<double>{0.5, -0.5, 0.5, -0.5});  // w,x,y,z
    declare_parameter("fov_half_angle_norm", 0.588836);  // model image bound
    declare_parameter("max_range", 0.0);   // 0 => no range gating
    declare_parameter("world_frame", std::string("odom"));

    publish_rate_ = get_parameter("publish_rate").as_double();
    const auto t = get_parameter("t_B_C").as_double_array();
    if (t.size() == 3) t_B_C_ = Eigen::Vector3d(t[0], t[1], t[2]);
    const auto q = get_parameter("q_B_M").as_double_array();
    if (q.size() == 4) q_B_M_ = Eigen::Quaterniond(q[0], q[1], q[2], q[3]);  // w,x,y,z
    q_B_M_.normalize();
    fov_half_ = get_parameter("fov_half_angle_norm").as_double();
    max_range_ = get_parameter("max_range").as_double();
    world_frame_ = get_parameter("world_frame").as_string();

    pub_bearing_ = create_publisher<fsc_vision_msgs::msg::TargetBearing>(
        "target_bearing", 10);
    pub_point_ = create_publisher<geometry_msgs::msg::PointStamped>(
        "target_point", 10);

    const rclcpp::QoS sensor_qos = rclcpp::SensorDataQoS();
    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
        "odometry", sensor_qos,
        std::bind(&TargetBearingNode::odomCallback, this, std::placeholders::_1));
    sub_target_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "target_pose", sensor_qos,
        std::bind(&TargetBearingNode::targetCallback, this, std::placeholders::_1));
    sub_gimbal_ = create_subscription<std_msgs::msg::Float64>(
        "gimbal_state", 10,
        std::bind(&TargetBearingNode::gimbalCallback, this, std::placeholders::_1));

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_);
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&TargetBearingNode::step, this));

    RCLCPP_INFO(get_logger(), "target_bearing_node up at %.1f Hz.", publish_rate_);
  }

 private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(mutex_);
    drone_pos_ = Eigen::Vector3d(msg->pose.pose.position.x,
                                 msg->pose.pose.position.y,
                                 msg->pose.pose.position.z);
    q_W_B_ = Eigen::Quaterniond(msg->pose.pose.orientation.w,
                                msg->pose.pose.orientation.x,
                                msg->pose.pose.orientation.y,
                                msg->pose.pose.orientation.z);
    have_odom_ = true;
  }

  void targetCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(mutex_);
    target_pos_ = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y,
                                  msg->pose.position.z);
    have_target_ = true;
  }

  void gimbalCallback(const std_msgs::msg::Float64::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(mutex_);
    theta_g_ = msg->data;
  }

  void step() {
    Eigen::Vector3d drone_pos, target_pos;
    Eigen::Quaterniond q_W_B;
    double theta_g;
    {
      std::lock_guard<std::mutex> lk(mutex_);
      if (!have_odom_ || !have_target_) return;
      drone_pos = drone_pos_;
      target_pos = target_pos_;
      q_W_B = q_W_B_;
      theta_g = theta_g_;
    }

    // Gimbaled body->camera rotation: q_B_C = q_B_M (x) q_pitch(theta_g).
    const Eigen::Quaterniond q_pitch(std::cos(0.5 * theta_g), 0.0,
                                     std::sin(0.5 * theta_g), 0.0);
    const Eigen::Quaterniond q_B_C = (q_B_M_ * q_pitch).normalized();

    // Target in camera frame.
    const Eigen::Vector3d p_lb_b = q_W_B.inverse() * (target_pos - drone_pos);
    const Eigen::Vector3d p_lc_c = q_B_C.inverse() * (p_lb_b - t_B_C_);
    const double range = p_lc_c.norm();

    fsc_vision_msgs::msg::TargetBearing out;
    out.header.stamp = now();
    out.header.frame_id = world_frame_;
    out.range = range;

    if (range < 1e-6) {
      // Degenerate: camera on top of target. Emit identity bearing, invalid.
      out.bearing.w = 1.0;
      out.valid = false;
      pub_bearing_->publish(out);
      publishPoint(target_pos);
      return;
    }

    const Eigen::Vector3d dir = p_lc_c / range;
    const Eigen::Quaterniond f1 = quatFromTwoNormals(
        Eigen::Vector3d::UnitZ(), dir, Eigen::Vector3d::UnitX());
    out.bearing.w = f1.w();
    out.bearing.x = f1.x();
    out.bearing.y = f1.y();
    out.bearing.z = f1.z();

    // Validity: target in front of the camera and within the model's FOV
    // (normalized image coordinates u = x/z, v = y/z), plus optional range gate.
    bool valid = dir.z() > 1e-3;
    if (valid) {
      const double u = p_lc_c.x() / p_lc_c.z();
      const double v = p_lc_c.y() / p_lc_c.z();
      valid = std::abs(u) <= fov_half_ && std::abs(v) <= fov_half_;
    }
    if (valid && max_range_ > 0.0) valid = range <= max_range_;
    out.valid = valid;

    pub_bearing_->publish(out);
    publishPoint(target_pos);
  }

  void publishPoint(const Eigen::Vector3d& p) {
    geometry_msgs::msg::PointStamped pt;
    pt.header.stamp = now();
    pt.header.frame_id = world_frame_;
    pt.point.x = p.x();
    pt.point.y = p.y();
    pt.point.z = p.z();
    pub_point_->publish(pt);
  }

  double publish_rate_{30.0};
  Eigen::Vector3d t_B_C_{0.0, 0.0, 0.15};
  Eigen::Quaterniond q_B_M_{0.5, -0.5, 0.5, -0.5};
  double fov_half_{0.588836};
  double max_range_{0.0};
  std::string world_frame_{"odom"};

  std::mutex mutex_;
  Eigen::Vector3d drone_pos_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond q_W_B_{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d target_pos_{Eigen::Vector3d::Zero()};
  double theta_g_{0.0};
  bool have_odom_{false};
  bool have_target_{false};

  rclcpp::Publisher<fsc_vision_msgs::msg::TargetBearing>::SharedPtr pub_bearing_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_point_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_target_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_gimbal_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TargetBearingNode>());
  rclcpp::shutdown();
  return 0;
}
