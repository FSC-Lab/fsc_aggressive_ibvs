#include "vision_common/math_common.h"
#include <ros/duration.h>

namespace vision_common {

using namespace Eigen;
using namespace std;

double deg2rad(double deg) { return 0.017453292519943 * deg; }

double rad2deg(double rad) { return 57.295779513082323 * rad; }

Eigen::Vector2d project2d(const Eigen::Vector3d& v) {
  return v.head<2>() / v[2];
}

Eigen::Vector3d unproject2d(const Eigen::Vector2d& v) {
  return Eigen::Vector3d(v[0], v[1], 1.0);
}

Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
  Eigen::Matrix3d v_sqew;
  v_sqew << 0, -v[2], v[1], v[2], 0, -v[0], -v[1], v[0], 0;
  return v_sqew;
}

Eigen::Vector2d round2d(const Eigen::Vector2d& pt) {
  return Eigen::Vector2d(roundf(pt.x()), roundf(pt.y()));
}

Eigen::Vector3d round3d(const Eigen::Vector3d& pt) {
  return Eigen::Vector3d(roundf(pt.x()), roundf(pt.y()), roundf(pt.z()));
}

Eigen::Matrix3d angleAxisToRotationMatrix(const Eigen::Vector3d& rvec) {
  double angle = rvec.norm();
  if (angle == double(0)) {
    return Eigen::Matrix3d::Identity();
  }

  Eigen::Vector3d axis;
  axis = rvec.normalized();

  Eigen::Matrix3d rmat;
  rmat = Eigen::AngleAxis<double>(angle, axis);

  return rmat;
}

Eigen::Quaterniond angleAxisToQuaternion(const Eigen::Vector3d& rvec) {
  Eigen::Matrix3d rmat = angleAxisToRotationMatrix(rvec);

  return Eigen::Quaterniond(rmat);
}

Eigen::Vector3d rotationToAngleAxis(const Eigen::Matrix3d& rmat) {
  Eigen::AngleAxis<double> angleaxis;
  angleaxis.fromRotationMatrix(rmat);
  return angleaxis.angle() * angleaxis.axis();
}

Eigen::Vector3d quaternionToAngleAxis(const Eigen::Quaterniond& q) {
  Eigen::Vector3d rvec;
  Eigen::Matrix3d rmat = q.toRotationMatrix();

  Eigen::AngleAxis<double> angleaxis;
  angleaxis.fromRotationMatrix(rmat);
  rvec = angleaxis.angle() * angleaxis.axis();
  return rvec;
}

Eigen::Vector3d getAngleAxisFromTwoNormals(const Eigen::Vector3d& a,
                                           const Eigen::Vector3d& b,
                                           const Eigen::Vector3d& a_perp) {
  Eigen::Vector3d rvec;
  const Eigen::Vector3d cross = a.cross(b);
  const double crossNorm = cross.norm();
  const double c = a.dot(b);
  const double angle = std::acos(c);
  if (crossNorm < 1e-6) {
    if (c > 0) {
      rvec = cross;
    } else {
      rvec = a_perp * M_PI;
    }
  } else {
    rvec = cross * (angle / crossNorm);
  }
  return rvec;
}

Eigen::Vector3d quaternionToEulerAnglesZYX(const Eigen::Quaterniond& q) {
  Eigen::Vector3d euler_angles;
  euler_angles(0) =
      atan2(2.0 * q.w() * q.x() + 2.0 * q.y() * q.z(),
            q.w() * q.w() - q.x() * q.x() - q.y() * q.y() + q.z() * q.z());
  euler_angles(1) = -asin(2.0 * q.x() * q.z() - 2.0 * q.w() * q.y());
  euler_angles(2) =
      atan2(2.0 * q.w() * q.z() + 2.0 * q.x() * q.y(),
            q.w() * q.w() + q.x() * q.x() - q.y() * q.y() - q.z() * q.z());
  return euler_angles;
}

Eigen::Quaterniond eulerAnglesZYXToQuaternion(
    const Eigen::Vector3d& euler_angles) {
  Eigen::Quaterniond q;
  double r = euler_angles(0) / 2.0;
  double p = euler_angles(1) / 2.0;
  double y = euler_angles(2) / 2.0;
  q.w() = cos(r) * cos(p) * cos(y) + sin(r) * sin(p) * sin(y);
  q.x() = sin(r) * cos(p) * cos(y) - cos(r) * sin(p) * sin(y);
  q.y() = cos(r) * sin(p) * cos(y) + sin(r) * cos(p) * sin(y);
  q.z() = cos(r) * cos(p) * sin(y) - sin(r) * sin(p) * cos(y);
  return q;
}

Eigen::Vector3d rotationMatrixToEulerAnglesZYX(const Eigen::Matrix3d& R) {
  Eigen::Vector3d euler_angles;
  euler_angles(0) = atan2(R(2, 1), R(2, 2));
  euler_angles(1) =
      -atan2(R(2, 0), sqrt(pow(R(2, 1), 2.0) + pow(R(2, 2), 2.0)));
  euler_angles(2) = atan2(R(1, 0), R(0, 0));
  return euler_angles;
}

Eigen::Matrix3d eulerAnglesZYXToRotationMatrix(
    const Eigen::Vector3d& euler_angles) {
  double r = euler_angles(0);
  double p = euler_angles(1);
  double y = euler_angles(2);

  Eigen::Matrix3d R;
  R(0, 0) = cos(y) * cos(p);
  R(1, 0) = sin(y) * cos(p);
  R(2, 0) = -sin(p);

  R(0, 1) = cos(y) * sin(p) * sin(r) - sin(y) * cos(r);
  R(1, 1) = sin(y) * sin(p) * sin(r) + cos(y) * cos(r);
  R(2, 1) = cos(p) * sin(r);

  R(0, 2) = cos(y) * sin(p) * cos(r) + sin(y) * sin(r);
  R(1, 2) = sin(y) * sin(p) * cos(r) - cos(y) * sin(r);
  R(2, 2) = cos(p) * cos(r);

  return R;
}

Eigen::Matrix3d quaternionToRotationMatrix(const Eigen::Quaterniond& q) {
  Eigen::Matrix3d R;

  R(0, 0) = q.w() * q.w() + q.x() * q.x() - q.y() * q.y() - q.z() * q.z();
  R(1, 0) = 2.0 * q.w() * q.z() + 2.0 * q.x() * q.y();
  R(2, 0) = 2.0 * q.x() * q.z() - 2.0 * q.w() * q.y();

  R(0, 1) = 2.0 * q.x() * q.y() - 2.0 * q.w() * q.z();
  R(1, 1) = q.w() * q.w() - q.x() * q.x() + q.y() * q.y() - q.z() * q.z();
  R(2, 1) = 2.0 * q.w() * q.x() + 2.0 * q.y() * q.z();

  R(0, 2) = 2.0 * q.w() * q.y() + 2.0 * q.x() * q.z();
  R(1, 2) = 2.0 * q.y() * q.z() - 2.0 * q.w() * q.x();
  R(2, 2) = q.w() * q.w() - q.x() * q.x() - q.y() * q.y() + q.z() * q.z();

  return R;
}

Eigen::Quaterniond RotationMatrixToQuaternion(const Eigen::Matrix3d& R) {
  return Eigen::Quaterniond(R);
}

Eigen::Vector3d quaternionRatesToBodyRates(const Eigen::Quaterniond& q2,
                                           const Eigen::Quaterniond& q1,
                                           const double dt,
                                           const bool rates_in_body_frame) {
  // Computes the angular velocity in body coordinate system for a rotation
  // from q1 to q2 within the time dt

  Eigen::Quaterniond q_dot;

  // Note, that the space of unit quaternion S(3) double covers the space
  // of physical attitudes SO(3) therefore q = -q.
  // I want the minimal q_dot, therefore I need to check which is
  // better q or -q (they both represent the same attitude in space)

  Eigen::Quaterniond q2_best = q2;
  if ((-q2_best.coeffs() - q1.coeffs()).norm() <
      (q2_best.coeffs() - q1.coeffs()).norm()) {
    q2_best.coeffs() = -q2_best.coeffs();
  }
  q_dot.coeffs() = (q2_best.coeffs() - q1.coeffs()) / dt;

  // [ o W_omega ]' = 2q_dot * q_bar in world frame
  // [ o B_omega ]' = 2q_bar * q_dot in body frame
  if (rates_in_body_frame) {
    return Eigen::Vector3d(2.0 * (q2_best.conjugate() * q_dot).vec());
  } else {
    return Eigen::Vector3d(2.0 * (q_dot * q2_best.conjugate()).vec());
  }
}

Eigen::Vector3d quaternionDeltaToBodyRates(const Eigen::Quaterniond& q2,
                                           const Eigen::Quaterniond& q1,
                                           const double dt,
                                           const bool rates_in_body_frame) {
  // Computes the angular velocity in body coordinate system for a rotation
  // from q1 to q2 within the time dt
  // This is basically the inverse of the integrateQuaternion function

  Eigen::Quaterniond q_e = q1.inverse() * q2;

  if (q_e.w() < 0) {
    // Make sure real part of quaternion has positive sign such that we take
    // the minimum distance from q1 to q2 to compute the body rates
    q_e = Eigen::Quaterniond(-q_e.w(), -q_e.x(), -q_e.y(), -q_e.z());
  }

  double a = sinc(acos(q_e.w()) / M_PI);

  Eigen::Vector3d alpha(q_e.x() / a, q_e.y() / a, q_e.z() / a);

  Eigen::Vector3d omega = 2.0 * alpha / dt;

  if (rates_in_body_frame) {
    return omega;
  } else {
    return q2 * omega;
  }
}

Eigen::Quaterniond integrateQuaternion(const Eigen::Quaterniond& q_start,
                                       const Eigen::Vector3d& bodyrates,
                                       const double dt) {
  // Pushes the orientation forward in time assuming constant bodyrates
  // B_omega_WB. Make sure that the bodyrates are in the body frame not the
  // world frame
  Eigen::Quaterniond q_end;

  double p = bodyrates.x();
  double q = bodyrates.y();
  double r = bodyrates.z();

  double omega_norm = bodyrates.norm();
  // only do something if the bodyrates are not euqal to zero
  if (omega_norm > 1e-40) {
    Eigen::Matrix4d Lambda;
    Lambda << 0, -p, -q, -r, p, 0, r, -q, q, -r, 0, p, r, q, -p, 0;
    Lambda = Lambda * 0.5;

    // here we use a hack because coeffs returns [x y z w ] but we always work
    // with [ w x y z ]
    Eigen::Vector4d q_start_as_vector(q_start.w(), q_start.x(), q_start.y(),
                                      q_start.z());  // [w x y z]
    Eigen::Vector4d q_end_as_vector;                 // [w x y z]

    q_end_as_vector =
        (Eigen::Matrix4d::Identity() * cos(omega_norm * dt / 2.0) +
         2.0 / omega_norm * Lambda * sin(omega_norm * dt / 2.0)) *
        q_start_as_vector;
    q_end = Eigen::Quaterniond(q_end_as_vector(0), q_end_as_vector(1),
                               q_end_as_vector(2), q_end_as_vector(3));
  } else {
    q_end = q_start;
  }
  return q_end;
}

double sinc(const double a) {
  if (fabs(a) >= 1e-5) {
    return sin(M_PI * a) / (M_PI * a);
  } else {
    return 1.0;
  }
}

double interpolate(const double v0, const double v1,
                   const double interpolation_ratio) {
  // interpolation_ratio = [0; 1]
  // interpolation_ratio = 0 -> get v0
  // interpolation_ratio = 1 -> get v1
  if (interpolation_ratio <= 0.0) {
    return v0;
  }
  if (interpolation_ratio >= 1.0) {
    return v1;
  }

  return v0 + interpolation_ratio * (v1 - v0);
}

Eigen::Vector3d interpolate(const Eigen::Vector3d& v0,
                            const Eigen::Vector3d& v1,
                            const double interpolation_ratio) {
  // interpolation_ratio = [0; 1]
  // interpolation_ratio = 0 -> get v0
  // interpolation_ratio = 1 -> get v1
  if (interpolation_ratio <= 0.0) {
    return v0;
  }
  if (interpolation_ratio >= 1.0) {
    return v1;
  }

  return v0 + interpolation_ratio * (v1 - v0);
}

Eigen::Quaterniond interpolate(const Eigen::Quaterniond& q0,
                               const Eigen::Quaterniond& q1,
                               const double interpolation_ratio) {
  // interpolation_ratio = [0; 1]
  // interpolation_ratio = 0 -> get q0
  // interpolation_ratio = 1 -> get q1
  if (interpolation_ratio <= 0.0) {
    return q0;
  }
  if (interpolation_ratio >= 1.0) {
    return q1;
  }

  return q0.slerp(interpolation_ratio, q1);
}


vision_common::TrajectoryPoint interpolate(
    const vision_common::TrajectoryPoint& p0,
    const vision_common::TrajectoryPoint& p1,
    const double interpolation_ratio) {
  // interpolation_ratio = [0; 1]
  // interpolation_ratio = 0 -> get p0
  // interpolation_ratio = 1 -> get p1
  if (interpolation_ratio <= 0.0) {
    return p0;
  }
  if (interpolation_ratio >= 1.0) {
    return p1;
  }

  vision_common::TrajectoryPoint p_inter;

  // Timestamp
  p_inter.time_from_start =
      p0.time_from_start +
      ros::Duration(interpolation_ratio *
                    (p1.time_from_start - p0.time_from_start).toSec());

  p_inter.length_from_start =
      p0.length_from_start +
      interpolation_ratio * (p1.length_from_start - p0.length_from_start);
  p_inter.tangent = interpolation_ratio * (p1.tangent - p0.tangent);
  
  p_inter.distance_to_gate =
      p0.distance_to_gate -
      interpolation_ratio * (p0.distance_to_gate - p1.distance_to_gate);

  p_inter.gate_id = p0.gate_id;

  // Pose
  p_inter.position = interpolate(p0.position, p1.position, interpolation_ratio);
  p_inter.orientation =
      interpolate(p0.orientation, p1.orientation, interpolation_ratio);

  // Linear derivatives
  p_inter.velocity = interpolate(p0.velocity, p1.velocity, interpolation_ratio);
  p_inter.acceleration =
      interpolate(p0.acceleration, p1.acceleration, interpolation_ratio);
  p_inter.jerk = interpolate(p0.jerk, p1.jerk, interpolation_ratio);
  p_inter.snap = interpolate(p0.snap, p1.snap, interpolation_ratio);

  // Angular derivatives
  p_inter.bodyrates =
      interpolate(p0.bodyrates, p1.bodyrates, interpolation_ratio);
  p_inter.angular_acceleration = interpolate(
      p0.angular_acceleration, p1.angular_acceleration, interpolation_ratio);
  p_inter.angular_jerk =
      interpolate(p0.angular_jerk, p1.angular_jerk, interpolation_ratio);
  p_inter.angular_snap =
      interpolate(p0.angular_snap, p1.angular_snap, interpolation_ratio);

  // Heading angle and its derivatives
  p_inter.heading = interpolate(p0.heading, p1.heading, interpolation_ratio);
  p_inter.heading_rate =
      interpolate(p0.heading_rate, p1.heading_rate, interpolation_ratio);
  p_inter.heading_acceleration = interpolate(
      p0.heading_acceleration, p1.heading_acceleration, interpolation_ratio);


  return p_inter;
}


double wrapZeroToTwoPi(const double angle) {
  if (angle >= 0.0 && angle <= 2.0 * M_PIl) {
    return angle;
  }
  double wrapped_angle = fmod(angle, 2.0 * M_PIl);
  if (wrapped_angle < 0.0) {
    wrapped_angle += 2.0 * M_PIl;
  }
  return wrapped_angle;
}

double wrapMinusPiToPi(const double angle) {
  if (angle >= -M_PIl && angle <= M_PIl) {
    return angle;
  }
  double wrapped_angle = angle + M_PIl;
  wrapped_angle = wrapZeroToTwoPi(wrapped_angle);
  wrapped_angle -= M_PIl;
  return wrapped_angle;
}

double wrapAngleDifference(const double current_angle,
                           const double desired_angle) {
  double angle_diff =
      wrapZeroToTwoPi(desired_angle) - wrapZeroToTwoPi(current_angle);
  if (angle_diff > M_PIl) {
    angle_diff = (-2.0 * M_PIl + angle_diff);
  }
  if (angle_diff < -M_PIl) {
    angle_diff = (2.0 * M_PIl + angle_diff);
  }
  return angle_diff;
}

void limit(double* val, const double min, const double max) {
  if (*val > max) {
    *val = max;
  }
  if (*val < min) {
    *val = min;
  }
}

Eigen::Vector3d normal(const Eigen::Vector3d& o, const Eigen::Vector3d& a,
                       const Eigen::Vector3d& b) {
  const Eigen::Vector3d lx = a - o;
  const Eigen::Vector3d ly = b - o;
  Eigen::Vector3d normal = lx.cross(ly);
  return normal.normalized();
}

double angle(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
  return vision_common::wrapMinusPiToPi(std::acos(a.dot(b)));
}

template <typename T>
int sgn(T val) {
  return (T(0) < val) - (val < T(0));
}

}  // namespace vision_common
