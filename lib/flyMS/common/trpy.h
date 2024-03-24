/// \file trpy.h for throttle-roll-pitch-yaw. This contains a base class for objects holding data relevant to throttle,
/// roll, pitch, and yaw
#pragma once

#include <exception>
#include <vector>

#include "flyMS/common/constants.h"
#include "flyMS/util/debug_mode.h"

namespace flyMS {

/**
 * @brief Generic class to hold some data that has a throttle, roll, pitch, and yaw axis
 *
 * @tparam T
 */
template <typename T>
class TRPY {
 public:
  /**
   * @brief Construct a new TRPY object. Data is initialized to zero
   *
   */
  TRPY() : values(std::vector<T>(4, static_cast<T>(0))) { check_args(); }

  /**
   * @brief Construct a new TRPY object by copying a vector
   *
   * @param values a vector of size 4 to copy from
   */
  explicit TRPY(const std::vector<T> &values) : values(values) { check_args(); }

  /**
   * @brief Construct a new TRPY object by consuming a vector
   *
   * @param values a vector of size 4 to consume
   */
  explicit TRPY(std::vector<T> &&values) : values(std::move(values)) { check_args(); }

  /**
   * @brief Get a reference to the throttle channel
   *
   * @return T& The throttle
   */
  T &throttle() { return values[kFLYMS_THROTTLE_INDEX]; }

  /**
   * @brief Get a referene to the throttle channel for a const instance
   *
   * @return const T&
   */
  const T &throttle() const { return values[kFLYMS_THROTTLE_INDEX]; }
  T &roll() { return values[kFLYMS_ROLL_INDEX]; }
  const T &roll() const { return values[kFLYMS_ROLL_INDEX]; }
  T &pitch() { return values[kFLYMS_PITCH_INDEX]; }
  const T &pitch() const { return values[kFLYMS_PITCH_INDEX]; }
  T &yaw() { return values[kFLYMS_YAW_INDEX]; }
  const T &yaw() const { return values[kFLYMS_YAW_INDEX]; }

  /**
   * @brief Get a reference to the underlying vector
   *
   * @return std::vector<T>&
   */
  std::vector<T> &vector() { return values; }
  const std::vector<T> &vector() const { return values; }

  /**
   * @brief Random access to the underlying vector
   *
   * @param index The index of the element desired. This must be less than 4
   * @return T&
   */
  T &operator[](uint32_t index) {
    if constexpr (kDEBUG_MODE) {
      if (index > 4) {
        throw std::runtime_error("Tried to index TRPY with something > 4");
      }
    }
    return values[index];
  }
  const T &operator[](uint32_t index) const {
    if constexpr (kDEBUG_MODE) {
      if (index > 4) {
        throw std::runtime_error("Tried to index TRPY with something > 4");
      }
    }
    return values[index];
  }

 private:
  std::vector<T> values;  //< The underlying data

  /**
   * @brief Check function that ensures the input is the right size. This is only run in debug mode.
   *
   */
  inline void check_args() {
    if constexpr (kDEBUG_MODE) {
      if (values.size() != 4) {
        throw std::invalid_argument("values must have 4 arguments");
      }
    }
  }
};

}  // namespace flyMS
