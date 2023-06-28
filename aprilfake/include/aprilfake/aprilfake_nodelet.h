#pragma once

#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <vision_common/conversion_common.h>
#include <vision_common/math_common.h>
#include <vision_common/parameter_helper.h>

#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include <mutex>
#include <thread>

#include <trajectory_generation_helper/heading_trajectory_helper.h>
#include <trajectory_generation_helper/polynomial_trajectory_helper.h>
#include <vision_common/state_estimate.h>
#include <vision_common/trajectory.h>
#include <vision_common/trajectory_point.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>

namespace aprilfake {

class AprilFakeNodelet : public nodelet::Nodelet {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  AprilFakeNodelet();
  virtual ~AprilFakeNodelet();
  virtual void onInit();
  void initializeNodelet();

 private:
  void timerCallback(const ros::TimerEvent& event);
  void imageCallback(const sensor_msgs::ImageConstPtr& image_msg,
                     const sensor_msgs::CameraInfoConstPtr& camera_info);
  void odometryCallback(const nav_msgs::Odometry& msg);
  void goalCallback(const geometry_msgs::PoseStamped& msg);
  void triggerCallback(const std_msgs::Empty& msg);
  Eigen::Vector3d projToCam(
    const Eigen::Vector3d& target,
    const Eigen::Vector3d& pos,
    const Eigen::Quaterniond& orient,
    const bool& add_bias = false);
  Eigen::Vector3d projToImg(const Eigen::Vector3d& point);

  void drawPoint(cv_bridge::CvImagePtr image_ptr,
                 const Eigen::Vector3d& point);
  void publishPoint(const Eigen::Vector3d& point, const Eigen::Vector3d& position);
  void publishTarget();
  visualization_msgs::Marker makeMarker(
    const Eigen::Vector3d& pt,
    const Eigen::Vector3d& color);
  visualization_msgs::MarkerArray makeMarkerArray(
      const vision_common::Trajectory& trajectory,
      const Eigen::Vector3d& color);

  bool isInImage(const Eigen::Vector3d& px);
  bool loadParameters();

  Eigen::Matrix4d getRelativeTransform(
      std::vector<cv::Point3d > objectPoints,
      std::vector<cv::Point2d > imagePoints,
      double fx, double fy, double cx, double cy);

 private:
  /* ROS Utils */
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber odom_sub_;
  ros::Subscriber goal_sub_;
  ros::Subscriber trig_sub_;

  ros::Publisher odom_pub_;
  ros::Publisher pose_pub_;
  ros::Publisher point_pub_;
  ros::Publisher true_point_pub_;
  ros::Publisher feature_pub_;
  ros::Publisher vis_target_pub_;
  ros::Publisher vis_traj_pub_;

  std::mutex main_mutex_;
  ros::Timer timer_;
  ros::CallbackQueue timer_queue_;
  std::unique_ptr<ros::AsyncSpinner> timer_spinner_;

  std::shared_ptr<image_transport::ImageTransport> it_;
  image_transport::CameraSubscriber image_sub_;
  image_transport::Publisher image_pub_;
  cv_bridge::CvImagePtr cv_image_;

  image_geometry::PinholeCameraModel camera_model_;
  double width_;
  double height_;

  bool camera_info_available_;
  bool image_available_ = false;
  bool odom_available_ = false;
  bool goal_received_;
  bool trig_received_;

  vision_common::StateEstimate drone_state_;

  vision_common::Trajectory target_trajectory_;
  vision_common::TrajectoryPoint target_state_;
  vision_common::TrajectoryPoint goal_state_;

  // corners:
  Eigen::Quaterniond R_;
  Eigen::MatrixXd polygen_;
  Eigen::MatrixXd image_points_;
  double size_;

  // parameters
  Eigen::Quaterniond q_B_C_;
  Eigen::Vector3d t_B_C_;

  int order_ = 5;
  double max_vel_ = 3.0;
  double max_thrust_ = 20.0;
  double max_roll_pitch_rate_ = 2.0;

  const double feature_publish_frequency_ = 30.0;

  bool add_bias_ = false;

};

}  // namespace aprilfake
