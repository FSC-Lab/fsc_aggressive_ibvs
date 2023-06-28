#include "vision_common/normal_vector.h"

namespace vision_common {

NormalVector::NormalVector()
    : orientation(Eigen::Quaterniond::Identity()),
      e_x(1, 0, 0),
      e_y(0, 1, 0),
      e_z(0, 0, 1) {}

NormalVector::NormalVector(const Eigen::Quaterniond& q)
    : orientation(q), e_x(1, 0, 0), e_y(0, 1, 0), e_z(0, 0, 1) {}

NormalVector::NormalVector(const Eigen::Vector3d& vec)
    : e_x(1, 0, 0), e_y(0, 1, 0), e_z(0, 0, 1) {
  setFromVector(vec);
}

NormalVector::NormalVector(const NormalVector& other)
    : e_x(1, 0, 0), e_y(0, 1, 0), e_z(0, 0, 1) {
  orientation = other.orientation;
}

NormalVector::~NormalVector() {}

bool NormalVector::isValid() const {
  if (std::isnan(orientation.norm())) {
    return false;
  }

  return true;
}

void NormalVector::setFromVector(const Eigen::Vector3d& vec) {
  const double d = vec.norm();
  if (d > 1e-6) {
    Eigen::Vector3d vec_normalized = vec / d;
    orientation = angleAxisToQuaternion(
        vision_common::getAngleAxisFromTwoNormals(e_z, vec_normalized, e_x));
  } else {
    orientation.setIdentity();
  }
}

void NormalVector::boxPlus(const Eigen::Vector2d& vecIn,
                           NormalVector& stateOut) const {
  Eigen::Quaterniond q =
      angleAxisToQuaternion(vecIn(0) * getPerp1() + vecIn(1) * getPerp2());
  stateOut.orientation =
      q * orientation;  // R_cl*q, c is world frame. use right update
}

void NormalVector::boxMinus(const NormalVector& stateIn,
                            Eigen::Vector2d& vecOut) const {
  vecOut =
      stateIn.getN().transpose() * getRotationFromTwoNormals(stateIn, *this);
}

Eigen::Matrix<double, 3, 2> NormalVector::getN() const {
  Eigen::Matrix<double, 3, 2> M;
  M.col(0) = getPerp1();
  M.col(1) = getPerp2();
  return M;
}

}  // namespace vision_common