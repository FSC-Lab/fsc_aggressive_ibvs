#pragma once

#include "vision_common/math_common.h"
#include "vision_common/conversion_common.h"

namespace vision_common {

struct NormalVector {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  NormalVector();
  NormalVector(const Eigen::Quaterniond& q);
  NormalVector(const Eigen::Vector3d& vec);
  NormalVector(const NormalVector& other);

  virtual ~NormalVector();

  bool isValid() const;

  void setFromVector(const Eigen::Vector3d& vec);
  void boxPlus(const Eigen::Vector2d& vecIn, NormalVector& stateOut) const;
  void boxMinus(const NormalVector& stateIn, Eigen::Vector2d& vecOut) const;
  Eigen::Matrix<double, 3, 2> getN() const;

  inline void setIdentity() { orientation.setIdentity(); }
  inline Eigen::Vector3d getVec() const { return orientation * e_z; }
  inline Eigen::Vector3d getPerp1() const { return orientation * e_x; }
  inline Eigen::Vector3d getPerp2() const { return orientation * e_y; }
  inline void rotated(const Eigen::Quaterniond& dq) {
    // R_cl1 = R_cl0 * dq
    orientation = dq * orientation;
  }
  NormalVector& operator=(const NormalVector& other) {
    orientation = other.orientation;
    return *this;
  }

  static Eigen::Vector3d getRotationFromTwoNormals(const NormalVector& a,
                                                   const NormalVector& b) {
    return vision_common::getAngleAxisFromTwoNormals(a.getVec(), b.getVec(),
                                                   a.getPerp1());
  }

  Eigen::Quaterniond orientation;  // R_cl
  const Eigen::Vector3d e_x;
  const Eigen::Vector3d e_y;
  const Eigen::Vector3d e_z;
};

}  // namespace vision_common