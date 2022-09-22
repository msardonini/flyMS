/**
 * @file vio_data.h
 * @author Mike Sardonini (msardonini@gmail.com)
 * @brief Declaration of VioData struct
 * @version 0.1
 * @date 2022-09-21
 *
 * @copyright Copyright (c) 2022
 *
 */
#pragma once

#include "Eigen/Dense"

struct VioData {
  VioData()
      : position(Eigen::Vector3f::Zero()), velocity(Eigen::Vector3f::Zero()), quat(Eigen::Quaternionf::Identity()) {}

  Eigen::Vector3f position;
  Eigen::Vector3f velocity;
  Eigen::Quaternionf quat;
};
