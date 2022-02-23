#pragma once

#include "Eigen/Dense"

struct VioData {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3f position;
  Eigen::Vector3f velocity;
  Eigen::Quaternionf quat;
};
