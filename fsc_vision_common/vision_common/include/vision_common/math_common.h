#pragma once

#include <Eigen/Dense>
#include "vision_common/trajectory_point.h"

namespace vision_common {

using namespace Eigen;
using namespace std;

double deg2rad(double deg);
double rad2deg(double rad);
Eigen::Vector2d project2d(const Eigen::Vector3d& v);
Eigen::Vector3d unproject2d(const Eigen::Vector2d& v);
Eigen::Matrix3d skew(const Eigen::Vector3d& v);
Eigen::Vector2d round2d(const Eigen::Vector2d& pt);
Eigen::Vector3d round3d(const Eigen::Vector3d& pt);
Eigen::Matrix3d angleAxisToRotationMatrix(const Eigen::Vector3d& rvec);
Eigen::Quaterniond angleAxisToQuaternion(const Eigen::Vector3d& rvec);
Eigen::Vector3d rotationToAngleAxis(const Eigen::Matrix3d& rmat);
Eigen::Vector3d quaternionToAngleAxis(const Eigen::Quaterniond& q);
Eigen::Vector3d getAngleAxisFromTwoNormals(const Eigen::Vector3d& a,
                                           const Eigen::Vector3d& b,
                                           const Eigen::Vector3d& a_perp);
Eigen::Vector3d quaternionToEulerAnglesZYX(const Eigen::Quaterniond& q);
Eigen::Quaterniond eulerAnglesZYXToQuaternion(
    const Eigen::Vector3d& euler_angles);
Eigen::Vector3d rotationMatrixToEulerAnglesZYX(const Eigen::Matrix3d& R);
Eigen::Matrix3d eulerAnglesZYXToRotationMatrix(
    const Eigen::Vector3d& euler_angles);
Eigen::Matrix3d quaternionToRotationMatrix(const Eigen::Quaterniond& q);
Eigen::Quaterniond RotationMatrixToQuaternion(const Eigen::Matrix3d& R);
Eigen::Vector3d quaternionRatesToBodyRates(
    const Eigen::Quaterniond& q2, const Eigen::Quaterniond& q1, const double dt,
    const bool rates_in_body_frame = true);
Eigen::Vector3d quaternionDeltaToBodyRates(
    const Eigen::Quaterniond& q2, const Eigen::Quaterniond& q1, const double dt,
    const bool rates_in_body_frame = true);
Eigen::Quaterniond integrateQuaternion(const Eigen::Quaterniond& q_start,
                                       const Eigen::Vector3d& bodyrates,
                                       const double dt);
double sinc(const double a);

double interpolate(const double v0, const double v1,
                   const double interpolation_ratio);
Eigen::Vector3d interpolate(const Eigen::Vector3d& v0,
                            const Eigen::Vector3d& v1,
                            const double interpolation_ratio);
Eigen::Quaterniond interpolate(const Eigen::Quaterniond& q0,
                               const Eigen::Quaterniond& q1,
                               const double interpolation_ratio);
vision_common::TrajectoryPoint interpolate(
    const vision_common::TrajectoryPoint& p0,
    const vision_common::TrajectoryPoint& p1,
    const double interpolation_ratio);                          
double wrapZeroToTwoPi(const double angle);
double wrapMinusPiToPi(const double angle);
double wrapAngleDifference(const double current_angle,
                           const double desired_angle);
void limit(double* val, const double min, const double max);
Eigen::Vector3d normal(const Eigen::Vector3d& o,
                       const Eigen::Vector3d& a,
                       const Eigen::Vector3d& b); 
double angle(const Eigen::Vector3d& a,
             const Eigen::Vector3d& b);

template <typename T>
int sgn(T val);

}  // namespace vision_common