#include "aprilfake/aprilfake_nodelet.h"

namespace aprilfake {

AprilFakeNodelet::AprilFakeNodelet() {}

AprilFakeNodelet::~AprilFakeNodelet() {}

void AprilFakeNodelet::onInit() {
  initializeNodelet();

  if (!loadParameters()) {
    ROS_ERROR("Fail to load controller parameters!");
    return;
  }
}

void AprilFakeNodelet::initializeNodelet() {
  nh_ = ros::NodeHandle("");
  pnh_ = ros::NodeHandle("~");

  it_ = std::shared_ptr<image_transport::ImageTransport>(
      new image_transport::ImageTransport(nh_));
  image_sub_ = it_->subscribeCamera("image_topic", 1,
                                    &AprilFakeNodelet::imageCallback, this);

  odom_sub_ =
      nh_.subscribe("odom_topic", 1, &AprilFakeNodelet::odometryCallback, this);

  goal_sub_ = nh_.subscribe(
      "goal_topic", 1, &AprilFakeNodelet::goalCallback, this);

  trig_sub_  = nh_.subscribe("trig_topic", 1, &AprilFakeNodelet::triggerCallback, this);

  image_pub_ = it_->advertise("point_image", 1);

  odom_pub_ = nh_.advertise<nav_msgs::Odometry>("target_odom_topic", 1);

  pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("target_pose_topic", 1); 

  point_pub_ = nh_.advertise<sensor_msgs::PointCloud>("point_topic", 1); 

  true_point_pub_ = nh_.advertise<sensor_msgs::PointCloud>("true_point_topic", 1); 

  feature_pub_ = nh_.advertise<geometry_msgs::Point>("feature_topic", 1); 

  vis_target_pub_ = nh_.advertise<visualization_msgs::Marker>("vis_target", 1);

  vis_traj_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("vis_trajectory", 1);

  ros::TimerOptions timer_pub_options(
      ros::Duration(1.0 / feature_publish_frequency_),
      boost::bind(&AprilFakeNodelet::timerCallback, this, _1),
      &timer_queue_);
  timer_ = nh_.createTimer(timer_pub_options);

  timer_spinner_.reset(new ros::AsyncSpinner(1, &timer_queue_));
  timer_spinner_->start();
}

void AprilFakeNodelet::timerCallback(const ros::TimerEvent& event) {
  std::lock_guard<std::mutex> lock(main_mutex_);

  //TODO:
  // 1. if there is no new goal, keep following the current one
  // 2. if there is no trajectory, directly publish current position
  // 3. if there is new goal, plan a trajectory
  // 4. if there is a triggre, set a specified trajectory

  if (goal_received_) {
    target_trajectory_ =
        trajectory_generation_helper::polynomials::computeTimeOptimalTrajectory(
            target_state_, goal_state_, order_, max_vel_, max_thrust_,
            max_roll_pitch_rate_, feature_publish_frequency_);

    if (target_trajectory_.trajectory_type != vision_common::Trajectory::TrajectoryType::GENERAL) {
      ROS_WARN("[%s] Target trajectory planning fails.", pnh_.getNamespace().c_str());
      goal_received_ = false;
      return;  
    }

    trajectory_generation_helper::heading::addConstantHeadingRate(
        target_state_.heading, goal_state_.heading, &target_trajectory_);

    vis_traj_pub_.publish(makeMarkerArray(target_trajectory_, Eigen::Vector3d(0.7,0.1,0.3)));

    goal_received_ = false;
  } else if (trig_received_) {
    Eigen::Vector3d p0(3,0,1);
    Eigen::Vector3d p1(6,5,1);
    Eigen::Vector3d p2(9,0,1);
    Eigen::Vector3d p3(12,-5,1);
    Eigen::Vector3d p4(15,0,1);
    Eigen::Vector3d p5(18,5,1);
    Eigen::Vector3d p6(21,0,1);
    Eigen::Vector3d p7(24,-5,1);
    Eigen::Vector3d p8(27,0,1);

    target_state_.position = p0;
    goal_state_.position = p8;

    std::vector<Eigen::Vector3d> way_points;
    way_points.push_back(p1);
    way_points.push_back(p2);
    way_points.push_back(p3);
    way_points.push_back(p4);
    way_points.push_back(p5);
    way_points.push_back(p6);
    way_points.push_back(p7);

    Eigen::VectorXd segment_times =
        Eigen::VectorXd::Ones(int(way_points.size()) + 1);

    polynomial_trajectories::PolynomialTrajectorySettings trajectory_settings;
    Eigen::VectorXd minimization_weights(5);  // pos, vel, acc, jerk, snap
    minimization_weights << 0.0, 1.0, 1.0, 1.0, 1.0;
    trajectory_settings.minimization_weights = minimization_weights;
    trajectory_settings.continuity_order = 3;
    trajectory_settings.polynomial_order = 11;
    trajectory_settings.way_points = way_points;

    target_trajectory_ = trajectory_generation_helper::
        polynomials::generateMinimumSnapTrajectoryWithSegmentRefinement(
            segment_times, target_state_, goal_state_, trajectory_settings,
            max_vel_, max_thrust_, max_roll_pitch_rate_,
            feature_publish_frequency_);

    if (target_trajectory_.trajectory_type != vision_common::Trajectory::TrajectoryType::GENERAL) {
      ROS_WARN("[%s] Target trajectory planning fails.", pnh_.getNamespace().c_str());
      trig_received_ = false;
      return;  
    }

    trajectory_generation_helper::heading::addConstantHeadingRate(
        0.0, 0.0, &target_trajectory_);

    vis_traj_pub_.publish(makeMarkerArray(target_trajectory_, Eigen::Vector3d(0.7,0.1,0.3)));

    trig_received_ = false;
  }

  if (!target_trajectory_.points.empty()) {
    target_state_ = target_trajectory_.points.front();
    target_trajectory_.points.pop_front();
  }

  if (odom_available_) {
    publishTarget();
    odom_available_ = false;
  }

}

Eigen::Vector3d AprilFakeNodelet::projToCam(
    const Eigen::Vector3d& target,
    const Eigen::Vector3d& pos,
    const Eigen::Quaterniond& orient,
    const bool& add_bias) {
  Eigen::Vector3d p_lb_b = orient.inverse() * (target - pos);
  Eigen::Vector3d p_lc_c = q_B_C_.inverse() * (p_lb_b - t_B_C_);

  if (add_bias) {
    double distance = p_lc_c.norm();
    double bias = 0.0;
    double coeff = 0.02;
    if (distance <= 3) {
      bias = 0.0;
    } else {
      bias = coeff * (distance - 3)*(distance - 3);
    }

    distance += bias;

    p_lc_c = p_lc_c.normalized() * distance;
  }

  return p_lc_c;
}

Eigen::Vector3d AprilFakeNodelet::projToImg(const Eigen::Vector3d& point) {
  cv::Point2d uv_cv = camera_model_.project3dToPixel(vision_common::eigenToCv(point));
  Eigen::Vector2d px = vision_common::cvToEigen(uv_cv);
  return Eigen::Vector3d(px.x(), px.y(), 1.0);
}

void AprilFakeNodelet::publishPoint(
    const Eigen::Vector3d& point,
    const Eigen::Vector3d& position) {
  sensor_msgs::PointCloud msg;
  msg.header.stamp = ros::Time::now();

  geometry_msgs::Point32 point_msg;
  // local position estimate in camera frame
  point_msg.x = point.x();
  point_msg.y = point.y();
  point_msg.z = point.z();
  msg.points.push_back(point_msg);

  if (add_bias_) {
    Eigen::Vector3d p_W_BW = drone_state_.position;
    Eigen::Quaterniond q_W_B = drone_state_.orientation;
    Eigen::Vector3d p_B_LB = q_B_C_ * point + t_B_C_;
    Eigen::Vector3d bias_position= q_W_B * p_B_LB + p_W_BW; 

    geometry_msgs::Point32 global_point_msg;
    // global position estimate of the target point
    global_point_msg.x = bias_position.x();
    global_point_msg.y = bias_position.y();
    global_point_msg.z = bias_position.z();
    msg.points.push_back(global_point_msg);
    point_pub_.publish(msg);

  } else {
    geometry_msgs::Point32 global_point_msg;
    // global position estimate of the target point
    global_point_msg.x = position.x();
    global_point_msg.y = position.y();
    global_point_msg.z = position.z();
    msg.points.push_back(global_point_msg);

    double hsize = 0.5 * size_;

    Eigen::Vector3d s0, s1, s2, s3;
    s0 << -hsize, -hsize, 0;
    s1 <<  hsize, -hsize, 0;
    s2 <<  hsize,  hsize, 0;
    s3 << -hsize,  hsize, 0;
    polygen_.resize(3, 4);
    polygen_.col(0) = R_ * s0 + position;
    polygen_.col(1) = R_ * s1 + position;
    polygen_.col(2) = R_ * s2 + position;
    polygen_.col(3) = R_ * s3 + position;

    // TODO: add corner points
    for (int i = 0; i < 4; ++i) {
      point_msg.x = image_points_.col(i).x();
      point_msg.y = image_points_.col(i).y();
      point_msg.z = image_points_.col(i).z();
      msg.points.push_back(point_msg);    
    }

    point_pub_.publish(msg);
  }

  // publish image
  if (image_available_) {
    Eigen::Vector3d feature = projToImg(point);
    drawPoint(cv_image_, point);
    
    geometry_msgs::Point ftr_msg;
    ftr_msg.x = feature.x();
    ftr_msg.y = feature.y();
    ftr_msg.z = feature.z();
    feature_pub_.publish(ftr_msg);

    image_pub_.publish(cv_image_->toImageMsg());
    image_available_ = false;
  }

}

void AprilFakeNodelet::drawPoint(
    cv_bridge::CvImagePtr image_ptr,
    const Eigen::Vector3d& point) {
  // const double su_min = -0.75;
  // const double su_max = 0.75;
  // const double sv_min = -0.45;
  // const double sv_max = 0.45;
  // cv::Point3d rect_ll(su_min, sv_min, 1.0);
  // cv::Point3d rect_lr(su_max, sv_min, 1.0);
  // cv::Point3d rect_ur(su_max, sv_max, 1.0);
  // cv::Point3d rect_ul(su_min, sv_max, 1.0);

  // cv::Point2d px_ll = camera_model_.project3dToPixel(rect_ll);
  // cv::Point2d px_lr = camera_model_.project3dToPixel(rect_lr);
  // cv::Point2d px_ur = camera_model_.project3dToPixel(rect_ur);
  // cv::Point2d px_ul = camera_model_.project3dToPixel(rect_ul);

  double bound_ratio = 0.6;
  double lb_u = -width_*bound_ratio / 2 + camera_model_.cx();
  double ub_u =  width_*bound_ratio / 2 + camera_model_.cx();
  double lb_v = -height_*bound_ratio / 2 + camera_model_.cy();
  double ub_v =  height_*bound_ratio / 2 + camera_model_.cy();

  cv::Point2d px_ll(lb_u, ub_v);
  cv::Point2d px_lr(ub_u, ub_v);
  cv::Point2d px_ur(ub_u, lb_v);
  cv::Point2d px_ul(lb_u, lb_v);

  cv::Point3d pc_ll = camera_model_.projectPixelTo3dRay(px_ll);
  cv::Point3d pc_ur = camera_model_.projectPixelTo3dRay(px_ur);

  // std::cout << "pc_ll: " << pc_ll.x << ", " << pc_ll.y << ", " << pc_ll.z << std::endl;
  // std::cout << "px_ur: " << pc_ur.x << ", " << pc_ur.y << ", " << pc_ur.z << std::endl;

  // std::cout << "box width: " << std::fabs(px_lr.x - px_ll.x) << std::endl;
  // std::cout << "box height: " << std::fabs(px_ul.y - px_ll.y) << std::endl;

  cv::rectangle(image_ptr->image, px_ll, px_ur, cv::Scalar(0, 255, 0), cv::LINE_4);

  if (isInImage(projToImg(point))) {
    // draw a circle in the image as the feature
    cv::Point2d uv_cv = camera_model_.project3dToPixel(vision_common::eigenToCv(point));
    cv::circle(image_ptr->image, uv_cv, 18, cv::Scalar(255, 0, 0), cv::FILLED, cv::LINE_8);
  }
}

visualization_msgs::Marker AprilFakeNodelet::makeMarker(
    const Eigen::Vector3d& pt,
    const Eigen::Vector3d& color) {
  visualization_msgs::Marker msg;

  msg.id = 0;
  msg.header.frame_id = "world";
  msg.header.stamp = ros::Time::now();

  msg.type = visualization_msgs::Marker::SPHERE_LIST;
  msg.action = visualization_msgs::Marker::ADD;
  msg.pose.orientation.w = 1.0;

  msg.color.a = 1.0;
  msg.color.r = color.x();
  msg.color.g = color.y();
  msg.color.b = color.z();

  msg.scale.x = 0.2;
  msg.scale.y = 0.2;
  msg.scale.z = 0.2;

  geometry_msgs::Point point;
  point.x = pt.x();
  point.y = pt.y();
  point.z = pt.z();
  msg.points.push_back(point);

  for (int i = 0; i < 4; i++) {
    point.x = polygen_.col(i).x();
    point.y = polygen_.col(i).y();
    point.z = polygen_.col(i).z();
    msg.points.push_back(point);
  }

  return msg;
}

// visualization_msgs::Marker AprilFakeNodelet::makeMarker(
//     const Eigen::Vector3d& point,
//     const Eigen::Vector3d& color) {
//   visualization_msgs::Marker msg;
//   msg.header.frame_id = "world";
//   msg.header.stamp = ros::Time::now();

//   msg.type = visualization_msgs::Marker::SPHERE;
//   msg.action = visualization_msgs::Marker::ADD;

//   msg.pose.orientation.w = 1.0;
//   msg.scale.x = 0.3;
//   msg.scale.y = 0.3;
//   msg.scale.z = 0.3;
//   msg.color.a = 1.0;
//   msg.color.r = color.x();
//   msg.color.g = color.y();
//   msg.color.b = color.z();
//   msg.id = 0;
//   msg.points.clear();

//   msg.pose.position.x = point.x();
//   msg.pose.position.y = point.y();
//   msg.pose.position.z = point.z();  
//   msg.pose.orientation.w = 1.0;
//   msg.pose.orientation.x = 0.0;
//   msg.pose.orientation.y = 0.0;
//   msg.pose.orientation.z = 0.0;

//   return msg;
// }

visualization_msgs::MarkerArray AprilFakeNodelet::makeMarkerArray(
    const vision_common::Trajectory& trajectory,
    const Eigen::Vector3d& color) {
  visualization_msgs::MarkerArray array_msg;

  visualization_msgs::Marker msg;
  msg.header.frame_id = "world";
  msg.header.stamp = ros::Time::now();

  msg.type = visualization_msgs::Marker::SPHERE_LIST;
  msg.action = visualization_msgs::Marker::ADD;

  msg.pose.orientation.w = 1.0;
  msg.scale.x = 0.1;
  msg.scale.y = 0.1;
  msg.scale.z = 0.1;
  msg.color.a = 0.8;

  msg.points.clear();
  msg.id = 0;
  msg.color.r = color.x();
  msg.color.g = color.y();
  msg.color.b = color.z();
  auto it_pt = trajectory.points.begin();
  for (; it_pt != trajectory.points.end(); ++it_pt) {
    geometry_msgs::Point pt;
    pt.x = it_pt->position.x();
    pt.y = it_pt->position.y();
    pt.z = it_pt->position.z();
    msg.points.push_back(pt);
  }
  array_msg.markers.push_back(msg);

  return array_msg;
}

Eigen::Matrix4d AprilFakeNodelet::getRelativeTransform(
    std::vector<cv::Point3d > objectPoints,
    std::vector<cv::Point2d > imagePoints,
    double fx, double fy, double cx, double cy) {
  // perform Perspective-n-Point camera pose estimation using the
  // above 3D-2D point correspondences
  cv::Mat rvec, tvec;
  cv::Matx33d cameraMatrix(fx,  0, cx,
                           0,  fy, cy,
                           0,   0,  1);
  cv::Vec4f distCoeffs(0,0,0,0); // distortion coefficients
  // TODO Perhaps something like SOLVEPNP_EPNP would be faster? Would
  // need to first check WHAT is a bottleneck in this code, and only
  // do this if PnP solution is the bottleneck.
  cv::solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);
  cv::Matx33d R;
  cv::Rodrigues(rvec, R);
  Eigen::Matrix3d wRo;
  wRo << R(0,0), R(0,1), R(0,2), R(1,0), R(1,1), R(1,2), R(2,0), R(2,1), R(2,2);

  Eigen::Matrix4d T; // homogeneous transformation matrix
  T.topLeftCorner(3, 3) = wRo;
  T.col(3).head(3) <<
      tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2);
  T.row(3) << 0,0,0,1;
  return T;
}

void AprilFakeNodelet::publishTarget() {
  ros::Time now = ros::Time::now();
  // publish pose msg
  geometry_msgs::PoseStamped pose_msg;
  pose_msg.header.stamp = now;
  pose_msg.pose.position = vision_common::vectorToPoint(
      vision_common::eigenToGeometry(target_state_.position));
  target_state_.orientation = vision_common::eulerAnglesZYXToQuaternion(
    Eigen::Vector3d(0.0, 0.0, target_state_.heading));
  pose_msg.pose.orientation = vision_common::eigenToGeometry(target_state_.orientation);
  pose_pub_.publish(pose_msg);

  nav_msgs::Odometry odom_msg;
  odom_msg.header.stamp = now;
  odom_msg.header.frame_id = "world";
  odom_msg.child_frame_id = "aprilfake_target";

  odom_msg.pose.pose.position = vision_common::vectorToPoint(vision_common::eigenToGeometry(target_state_.position));
  odom_msg.pose.pose.orientation = vision_common::eigenToGeometry(target_state_.orientation);
  odom_msg.twist.twist.linear = vision_common::eigenToGeometry(target_state_.velocity);
  odom_pub_.publish(odom_msg);

  vis_target_pub_.publish(makeMarker(target_state_.position, Eigen::Vector3d(0.1,0.3,0.5)));

  // publish point msg
  Eigen::Vector3d point = projToCam(
    target_state_.position,
    drone_state_.position,
    drone_state_.orientation,
    add_bias_);

  // TODO: get point distance using PnP

  double hsize = 0.5 * size_;
  Eigen::Vector3d s0, s1, s2, s3;
  s0 << -hsize, -hsize, 0;
  s1 <<  hsize, -hsize, 0;
  s2 <<  hsize,  hsize, 0;
  s3 << -hsize,  hsize, 0;
  polygen_.resize(3, 4);
  polygen_.col(0) = R_ * s0 + target_state_.position;
  polygen_.col(1) = R_ * s1 + target_state_.position;
  polygen_.col(2) = R_ * s2 + target_state_.position;
  polygen_.col(3) = R_ * s3 + target_state_.position;

  std::vector<cv::Point3d> objectPoints;
  objectPoints.push_back(cv::Point3d(-hsize,-hsize, 0));
  objectPoints.push_back(cv::Point3d( hsize,-hsize, 0));
  objectPoints.push_back(cv::Point3d( hsize, hsize, 0));
  objectPoints.push_back(cv::Point3d(-hsize, hsize, 0));

  image_points_.resize(3, 4);
  std::vector<cv::Point2d> imagePoints;
  for (int i = 0; i < 4; ++i) {
    Eigen::Vector3d px = projToImg(projToCam(polygen_.col(i) ,
                                   drone_state_.position,
                                   drone_state_.orientation,
                                   add_bias_));
    image_points_.col(i) = px;        
    imagePoints.push_back(cv::Point2d(px.x(), px.y()));
  }


  Eigen::Matrix4d transform = getRelativeTransform(objectPoints,
                                                   imagePoints,
                                                   camera_model_.fx(),
                                                   camera_model_.fy(),
                                                   camera_model_.cx(),
                                                   camera_model_.cy());

  double distance_pnp = transform.col(3).head(3).norm();

  // std::cout << "true distance: " << point.norm() << std::endl;
  // std::cout << "pnp distance: " << distance_pnp << std::endl;

  point = point / point.norm() * distance_pnp;
  
  publishPoint(point, target_state_.position);


  Eigen::Vector3d true_point = projToCam(
    target_state_.position,
    drone_state_.position,
    drone_state_.orientation);

  sensor_msgs::PointCloud msg;
  msg.header.stamp = ros::Time::now();

  geometry_msgs::Point32 point_msg;
  // local position estimate in camera frame
  point_msg.x = true_point.x();
  point_msg.y = true_point.y();
  point_msg.z = true_point.z();
  msg.points.push_back(point_msg);

  point_msg.x = point.norm() - true_point.norm();
  point_msg.y = 0.0;
  point_msg.z = 0.0;
  msg.points.push_back(point_msg);

  true_point_pub_.publish(msg);
}

void AprilFakeNodelet::imageCallback(
    const sensor_msgs::ImageConstPtr& image_msg,
    const sensor_msgs::CameraInfoConstPtr& camera_info) {
  std::lock_guard<std::mutex> lock(main_mutex_);

  try {
    cv_image_ = cv_bridge::toCvCopy(image_msg, image_msg->encoding);
    image_available_ = true;
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  if (!camera_info_available_) {
    camera_model_.fromCameraInfo(camera_info);
    width_ = 2 * camera_model_.cx() - 1;
    height_ = 2 * camera_model_.cy() - 1;
    camera_info_available_ = true;
  }
}

void AprilFakeNodelet::odometryCallback(const nav_msgs::Odometry& msg) {
  std::lock_guard<std::mutex> lock(main_mutex_);
  
  drone_state_ = vision_common::StateEstimate(msg);
  odom_available_ = true;
}

void AprilFakeNodelet::goalCallback(const geometry_msgs::PoseStamped& msg) {
  std::lock_guard<std::mutex> lock(main_mutex_);
  
  Eigen::Vector3d xyz = vision_common::geometryToEigen(msg.pose.position);
  goal_state_.position << xyz.x(), xyz.y(), 1.0;

  goal_received_ = true;
}

void AprilFakeNodelet::triggerCallback(const std_msgs::Empty& msg) {
  std::lock_guard<std::mutex> lock(main_mutex_);

  trig_received_ = true;
}

bool AprilFakeNodelet::isInImage(const Eigen::Vector3d& px) {
  if (px.x() <= 0.0 || px.y() <= 0.0 || px.x() > width_ || px.y() > height_) {
    return false;
  }

  return true;
}

bool AprilFakeNodelet::loadParameters() {
// TODO: load parameters
#define GET_PARAM(name) \
  if (!vision_common::getParam(#name, name, pnh_)) return false

#define GET_PARAM_(name) \
  if (!vision_common::getParam(#name, name##_, pnh_)) return false

  GET_PARAM_(max_vel);
  GET_PARAM_(max_thrust);
  GET_PARAM_(max_roll_pitch_rate);
  GET_PARAM_(add_bias);
  GET_PARAM_(size);

  std::vector<double> t_B_C(3), q_B_C(4);
  if (!pnh_.getParam("t_B_C", t_B_C)) {
    ROS_WARN("Camera extrinsic translation is not set.");
    return false;
  } else {
    t_B_C_ << t_B_C[0], t_B_C[1], t_B_C[2];
  }
  if (!pnh_.getParam("q_B_C", q_B_C)) {
    ROS_WARN("Camera extrinsic rotation is not set.");
    return false;
  } else {
    q_B_C_ = Eigen::Quaterniond(q_B_C[0], q_B_C[1], q_B_C[2], q_B_C[3]);
  }

  std::vector<double> target_position(3), target_rotation(3);
  if (!pnh_.getParam("target_position", target_position)) {
    ROS_WARN("Feature Detector: landmark_position is not set.");
  } else {
    target_state_.position  << target_position[0],
                             target_position[1],
                             target_position[2];  
    target_state_.heading = 0.0;
    goal_state_ = target_state_;
  }

  if (!pnh_.getParam("target_rotation", target_rotation)) {
    ROS_WARN("Feature Detector: landmark_position is not set.");
  } else {
    double roll = vision_common::deg2rad(target_rotation[0]);
    double pitch = vision_common::deg2rad(target_rotation[1]);
    double yaw = vision_common::deg2rad(target_rotation[2]);
    R_ = vision_common::eulerAnglesZYXToQuaternion(Eigen::Vector3d(roll, pitch, yaw));

    double hsize = 0.5 * size_;

    Eigen::Vector3d s0, s1, s2, s3;
    s0 << -hsize, -hsize, 0;
    s1 <<  hsize, -hsize, 0;
    s2 <<  hsize,  hsize, 0;
    s3 << -hsize,  hsize, 0;
    polygen_.resize(3, 4);
    polygen_.col(0) = R_ * s0 + target_state_.position;
    polygen_.col(1) = R_ * s1 + target_state_.position;
    polygen_.col(2) = R_ * s2 + target_state_.position;
    polygen_.col(3) = R_ * s3 + target_state_.position;

  }

#undef GET_PARAM
#undef GET_PARAM_

  return true;
}

}  // namespace aprilfake

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aprilfake::AprilFakeNodelet, nodelet::Nodelet);
