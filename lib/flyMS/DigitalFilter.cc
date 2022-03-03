#include "flyMS/DigitalFilter.hpp"

#include <iostream>
#include <numeric>
#include <stdexcept>

#include "spdlog/spdlog.h"

namespace flyMS {

static constexpr double denominator_tol = 1.0E-8;

void DigitalFilter::check_args() {
  if (numerator_.size() < 1 || denominator_.size() < 1) {
    throw std::invalid_argument("Error in DigitalFilter. Numerator and Denominator must have >=2 values!");
  }

  if (numerator_.size() != denominator_.size()) {
    throw std::invalid_argument("Error in DigitalFilter. Numerator and Denominator have different sizes!");
  }

  if (std::abs(denominator_[0] - 1.) > denominator_tol) {
    spdlog::warn(
        "Digital Filter received non-zero leading term for Denominator Coefficients. The filter will be normalized");

    auto val = denominator_[0];
    std::for_each(numerator_.begin(), numerator_.end(), [val](auto num) { return num / val; });
    std::for_each(denominator_.begin(), denominator_.end(), [val](auto den) { return den / val; });
  }
}

DigitalFilter::DigitalFilter(const std::vector<double> &numerator, const std::vector<double> &denominator)
    : is_initialized_(true),
      numerator_(numerator),
      denominator_(denominator),
      filter_hist_(numerator.size(), 0.f),
      signal_hist_(numerator.size(), 0.f) {
  check_args();
}

void DigitalFilter::init(const std::vector<double> &numerator, const std::vector<double> &denominator) {
  numerator_ = numerator;
  denominator_ = denominator;
  filter_hist_ = std::vector<double>(numerator_.size(), 0.);
  signal_hist_ = std::vector<double>(numerator_.size(), 0.);

  check_args();
  is_initialized_ = true;
}

void DigitalFilter::zero_values() {
  std::for_each(signal_hist_.begin(), signal_hist_.end(), [](auto &val) { return 0.; });
  std::for_each(filter_hist_.begin(), filter_hist_.end(), [](auto &val) { return 0.; });
}

double DigitalFilter::update_filter(double val) {
  if (!is_initialized_) {
    throw std::runtime_error("Error! Tried to update non-initialized filter!");
  }

  signal_hist_[current_index_] = val;

  double filt_val = signal_hist_[current_index_] * numerator_.front();

  // Apply the difference equation in place, using std::accumulate
  auto signal_it = signal_hist_.begin() + current_index_ + 1;
  auto filter_it = filter_hist_.begin() + current_index_ + 1;
  auto den_it = denominator_.begin() + 1;
  filt_val += std::accumulate(numerator_.begin() + 1, numerator_.end() - current_index_, 0.,
                              [&](auto a, auto num) { return a + (num * *signal_it++) - (*den_it++ * *filter_it++); });

  signal_it = signal_hist_.begin();
  filter_it = filter_hist_.begin();
  den_it = denominator_.end() - current_index_;
  filt_val += std::accumulate(numerator_.end() - current_index_, numerator_.end(), 0.,
                              [&](auto a, auto num) { return a + (num * *signal_it++) - (*den_it++ * *filter_it++); });

  filter_hist_[current_index_] = filt_val;

  if (current_index_ == 0) {
    current_index_ = numerator_.size() - 1;
  } else {
    current_index_--;
  }
  return filt_val;
}

DigitalFilter generate_pid(double kp, double ki, double kd, double tf, double dt) {
  if (tf <= 2 * dt) {
    throw std::invalid_argument("tf must be > 2dt for stability");
  }
  // if ki==0, return a PD filter with rolloff
  if (ki == 0) {
    std::vector<double> numerator{(kp * tf + kd) / tf, -(((ki * dt - kp) * (dt - tf)) + kd) / tf};
    std::vector<double> denominator{1, -(tf - dt) / tf};
    return DigitalFilter(numerator, denominator);
  } else {
    std::vector<double> numerator{(kp * tf + kd) / tf, (ki * dt * tf + kp * (dt - tf) - kp * tf - 2.0 * kd) / tf,
                                  (((ki * dt - kp) * (dt - tf)) + kd) / tf};
    std::vector<double> denominator{1, (dt - (2.0 * tf)) / tf, (tf - dt) / tf};
    return DigitalFilter(numerator, denominator);
  }
}

DigitalFilter generate_pid(const std::array<double, 3> &pid_constants, double tf, double dt) {
  return generate_pid(pid_constants[0], pid_constants[1], pid_constants[2], tf, dt);
}
}  // namespace flyMS
