#pragma once

#include "Eigen/Dense"

struct VioData {
  VioData()
      : position(Eigen::Vector3f::Zero()), velocity(Eigen::Vector3f::Zero()), quat(Eigen::Quaternionf::Identity()) {}

  Eigen::Vector3f position;
  Eigen::Vector3f velocity;
  Eigen::Quaternionf quat;
};
