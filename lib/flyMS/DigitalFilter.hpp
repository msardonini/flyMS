#pragma once

#include <array>
#include <cstdint>
#include <deque>
#include <vector>

namespace flyMS {

class DigitalFilter {
 public:
  /**
   * @brief Construct a new Digital Filter object
   *
   * @param numerator Numerator coefficients. Length must match denominator
   * @param denominator Denominator coeffiecints. Leading term should be 1, the filter will normalize itself otherwise
   */
  DigitalFilter(const std::vector<double> &numerator, const std::vector<double> &denominator);

  /**
   * @brief Construct a new Digital Filter object
   *
   */
  DigitalFilter() = default;

  /**
   * @brief Initialize the object if created with default ctor
   *
   * @param numerator Numerator coefficients. Length must match denominator
   * @param denominator Denominator coeffiecints. Leading term should be 1, the filter will normalize itself otherwise
   */
  void init(const std::vector<double> &numerator, const std::vector<double> &denominator);

  /**
   * @brief Increment the filter and return the output
   *
   * @param val Input value
   * @return double Filtered value
   */
  double update_filter(double val);

  /**
   * @brief Zeros the values in the filter
   *
   */
  void zero_values();

 private:
  /**
   * @brief Checks the objects values to make sure they are as expected. Run on initialization
   *
   */
  void check_args();

  bool is_initialized_ = false;      //< Flag if the filter has been initialized
  std::uint8_t current_index_ = 0;   //< The current index. Needed for processing data in place
  std::vector<double> numerator_;    //< Numerator coefficients
  std::vector<double> denominator_;  //< Denominator coefficients
  std::vector<double> filter_hist_;  //< The recent values of the raw signal
  std::vector<double> signal_hist_;  //< The recent values of the filtered signal
};

/**
 * @brief Generates a filtered PID controller and returns it as a digital filter object.
 *
 * @param kp The proportional gain
 * @param ki The integrator gain
 * @param kd The derivative gain
 * @param tf The filtering coefficient. Must be > 2*dt
 * @param dt The time between updates, i.e. the inverse of the sample rate in seconds
 * @return DigitalFilter
 */
DigitalFilter generate_pid(double kp, double ki, double kd, double tf, double dt);

/**
 * @brief Overload of the generate_pid function, using std::array for pid constants
 *
 * @param pid_constants PID constants as an array
 * @param tf The filtering coefficient. Must be > 2*dt
 * @param dt The time between updates, i.e. the inverse of the sample rate in seconds
 * @return DigitalFilter
 */
DigitalFilter generate_pid(const std::array<double, 3> &pid_constants, double tf, double dt);

}  // namespace flyMS
