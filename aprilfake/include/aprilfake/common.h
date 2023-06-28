#pragma once

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

namespace aprilfake {

inline Eigen::Vector2d project2d(const Eigen::Vector3d& v) {
  return v.head<2>() / v[2];
}

inline Eigen::Vector3d unproject2d(const Eigen::Vector2d& v) {
  return Eigen::Vector3d(v[0], v[1], 1.0);
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

inline Eigen::Vector3d roundfEigen(const Eigen::Vector3d& pt) {
  return Eigen::Vector3d(roundf(pt.x()), roundf(pt.y()), roundf(pt.z()));
}

}