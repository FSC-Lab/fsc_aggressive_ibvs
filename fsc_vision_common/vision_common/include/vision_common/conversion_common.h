#pragma once

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>

namespace vision_common {
  
// Quaternions
inline Eigen::Quaterniond geometryToEigen(
    const geometry_msgs::Quaternion& vec_ros) {
  Eigen::Quaterniond vec_eigen;
  vec_eigen.x() = vec_ros.x;
  vec_eigen.y() = vec_ros.y;
  vec_eigen.z() = vec_ros.z;
  vec_eigen.w() = vec_ros.w;
  return vec_eigen;
}

inline geometry_msgs::Quaternion eigenToGeometry(
    const Eigen::Quaterniond& vec_eigen) {
  geometry_msgs::Quaternion vec_ros;
  vec_ros.x = vec_eigen.x();
  vec_ros.y = vec_eigen.y();
  vec_ros.z = vec_eigen.z();
  vec_ros.w = vec_eigen.w();
  return vec_ros;
}

// Vectors
inline Eigen::Vector3d geometryToEigen(const geometry_msgs::Vector3& vec_ros) {
  Eigen::Vector3d vec_eigen;
  vec_eigen.x() = vec_ros.x;
  vec_eigen.y() = vec_ros.y;
  vec_eigen.z() = vec_ros.z;
  return vec_eigen;
}

inline geometry_msgs::Vector3 eigenToGeometry(
    const Eigen::Vector3d& vec_eigen) {
  geometry_msgs::Vector3 vec_ros;
  vec_ros.x = vec_eigen.x();
  vec_ros.y = vec_eigen.y();
  vec_ros.z = vec_eigen.z();
  return vec_ros;
}

// Points
inline Eigen::Vector3d geometryToEigen(const geometry_msgs::Point& vec_ros) {
  Eigen::Vector3d vec_eigen;
  vec_eigen.x() = vec_ros.x;
  vec_eigen.y() = vec_ros.y;
  vec_eigen.z() = vec_ros.z;
  return vec_eigen;
}

inline geometry_msgs::Point vectorToPoint(
    const geometry_msgs::Vector3& vector) {
  geometry_msgs::Point point;
  point.x = vector.x;
  point.y = vector.y;
  point.z = vector.z;
  return point;
}

inline Eigen::Vector3d cvToEigen(const cv::Point3d& pt_cv) {
  return Eigen::Vector3d(pt_cv.x, pt_cv.y, pt_cv.z);
}

inline cv::Point3d eigenToCv(const Eigen::Vector3d& pt_eg) {
  return cv::Point3d(pt_eg.x(), pt_eg.y(), pt_eg.z());
}

inline Eigen::Vector2d cvToEigen(const cv::Point2d& pt_cv) {
  return Eigen::Vector2d(pt_cv.x, pt_cv.y);
}

inline cv::Point2d eigenToCv(const Eigen::Vector2d& pt_eg) {
  return cv::Point2d(pt_eg.x(), pt_eg.y());
}

}