/**
 * @file DigitalFilter.hpp
 * @author Mike Sardonini (msardonini@gmail.com)
 * @brief DigitalFilter class
 * @version 0.1
 * @date 2022-09-21
 *
 * @copyright Copyright (c) 2022
 *
 */
#pragma once

#include <array>
#include <cstdint>
#include <deque>
#include <vector>

namespace flyMS {

/**
 * @brief Class for digital filtering. Any discrete filter can be implemented, including FIR and IIR. Typical use cases
 * are for PID controllers, and frequency selective filters.
 *
 * Example Usage:
 * ```
 * // Create a simple integrator filter
 * std::vector<double> num = {0.0, 1.0};
 * std::vector<double> den = {1.0, -1.0};
 * flyMS::DigitalFilter filt(num, den);
 *  for (auto i = 0; i < 50; i++) {
 *    if (i == filt.update_filter(1.)) {
 *      std::cout << "Integrator works!" << std::endl;
 *    } else {
 *      std::cout << "Integrator failed!" << std::endl;
 *  }
 * ```
 */
class DigitalFilter {
 public:
  /**
   * @brief Construct a new Digital Filter object
   *
   * @param numerator Numerator coefficients. Length must match denominator
   * @param denominator Denominator coefficients. Leading term should be 1, the filter will normalize itself otherwise
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
   * @param denominator Denominator coefficients. Leading term should be 1, the filter will normalize itself otherwise
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

  /**
   * @brief Sets all the values of the filtered and unfiltered output states to a value. Helpful to avoid step inputs
   *
   * @param val
   */
  void set_to_value(double val);

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
