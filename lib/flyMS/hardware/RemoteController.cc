#include "flyMS/hardware/RemoteController.h"

#include <chrono>
#include <functional>

#include "rc/dsm.h"
#include "rc/start_stop.h"

namespace flyMS {

static constexpr uint32_t kCONNECTION_MONITOR_LOOP_FRQ = 10;
static constexpr auto kCONNECTION_MONITOR_LOOP_SLEEP_TIME_US =
    std::chrono::microseconds(1000000 / kCONNECTION_MONITOR_LOOP_FRQ);
static constexpr uint32_t kCONNECTION_MONITOR_TIMEOUT_NS = 1000000000;  //< 1 second

RemoteController& RemoteController::get_instance() {
  static RemoteController instance;
  return instance;
}

RemoteController::RemoteController() {
  if (rc_dsm_init() != 0) {
    throw std::runtime_error("Could not initialize remote controller!");
  }

  rc_dsm_set_callback(&RemoteController::data_callback);

  is_running_.store(true);
  connection_monitor_thread_ = std::thread(&RemoteController::lost_connection_monitor, this);
}

RemoteController::~RemoteController() {
  rc_dsm_cleanup();

  is_running_.store(false);
  if (connection_monitor_thread_.joinable()) {
    connection_monitor_thread_.join();
  }
}

bool RemoteController::has_received_data() const { return !rc_data_.empty(); }

std::vector<float> RemoteController::get_channel_values() {
  std::scoped_lock<std::mutex> lock(rc_mutex_);
  return rc_data_;
}

void RemoteController::data_callback() {
  std::vector<float> channel_data(RC_MAX_DSM_CHANNELS);
  for (auto i = 0; i < RC_MAX_DSM_CHANNELS; i++) {
    channel_data[i] = rc_dsm_ch_normalized(i);
  }
  auto& this_instance = RemoteController::get_instance();
  this_instance.set_channel_values(std::move(channel_data));
}

void RemoteController::set_channel_values(std::vector<float>&& values) {
  std::lock_guard<std::mutex> lock(rc_mutex_);
  rc_data_ = std::move(values);
}

void RemoteController::lost_connection_monitor() {
  while (is_running_.load()) {
    if (rc_dsm_nanos_since_last_packet() > kCONNECTION_MONITOR_TIMEOUT_NS) {
      rc_set_state(EXITING);
    }
    std::this_thread::sleep_for(kCONNECTION_MONITOR_LOOP_SLEEP_TIME_US);
  }
}

}  // namespace flyMS
