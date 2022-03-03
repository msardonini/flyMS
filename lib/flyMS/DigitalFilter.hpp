#pragma once

#include <array>
#include <cstdint>
#include <deque>
#include <vector>

namespace flyMS {

class DigitalFilter {
 public:
  DigitalFilter(const std::vector<double> &numerator, const std::vector<double> &denominator);

  DigitalFilter() = default;

  void init(const std::vector<double> &numerator, const std::vector<double> &denominator);

  double update_filter(double val);

  void zero_values();

 private:
  void check_args();
  bool is_initialized_ = false;
  std::uint8_t current_index_ = 0;
  std::vector<double> numerator_;
  std::vector<double> denominator_;
  std::vector<double> filter_hist_;
  std::vector<double> signal_hist_;
};

DigitalFilter generate_pid(double kp, double ki, double kd, double tf, double dt);

DigitalFilter generate_pid(const std::array<double, 3> &pid_constants, double tf, double dt);

}  // namespace flyMS
