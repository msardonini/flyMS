#include "flyMS/hardware/RemoteController.h"

#include <chrono>
#include <functional>

#include "rc/dsm.h"
#include "rc/start_stop.h"
#include "spdlog/spdlog.h"

namespace flyMS {

static constexpr uint32_t kCONNECTION_MONITOR_LOOP_FRQ = 10;
static constexpr auto kCONNECTION_MONITOR_LOOP_SLEEP_TIME =
    std::chrono::microseconds(1000000 / kCONNECTION_MONITOR_LOOP_FRQ);
static constexpr uint32_t kCONNECTION_MONITOR_TIMEOUT_NS = 1000000000;                      //< 1 second
static constexpr auto kWAIT_FOR_DATA_PACKET_SLEEP_TIME = std::chrono::microseconds(10000);  //< 10 ms, this is polled

RemoteController& RemoteController::get_instance() {
  static RemoteController instance;
  return instance;
}

RemoteController::RemoteController() {
  if (rc_dsm_init() != 0) {
    throw std::runtime_error("Could not initialize remote controller!");
  }

  rc_dsm_set_callback(&RemoteController::data_callback);
}

RemoteController::~RemoteController() {
  rc_dsm_cleanup();
  stop_packet_loss_monitoring();
}

bool RemoteController::has_received_data() const { return !rc_data_.empty(); }

bool RemoteController::wait_for_data_packet() {
  while (!has_received_data()) {
    if (rc_get_state() == EXITING) {
      return false;
    }
    std::this_thread::sleep_for(std::chrono::microseconds(kWAIT_FOR_DATA_PACKET_SLEEP_TIME));
  }
  return true;
}

std::vector<float> RemoteController::get_channel_data() {
  std::scoped_lock<std::mutex> lock(rc_mutex_);
  return rc_data_;
}

void RemoteController::data_callback() {
  std::vector<float> channel_data(RC_MAX_DSM_CHANNELS);
  for (auto i = 0; i < RC_MAX_DSM_CHANNELS; i++) {
    channel_data[i] = rc_dsm_ch_normalized(i + 1);
  }
  auto& this_instance = RemoteController::get_instance();
  this_instance.set_channel_values(std::move(channel_data));
}

void RemoteController::set_channel_values(std::vector<float>&& values) {
  std::lock_guard<std::mutex> lock(rc_mutex_);
  rc_data_ = std::move(values);
}

void RemoteController::packet_loss_callback(std::function<void()> callback) {
  packet_loss_callback_ = callback;

  is_running_ = true;
  connection_monitor_thread_ = std::thread(&RemoteController::lost_connection_monitor, this);
}

void RemoteController::stop_packet_loss_monitoring() {
  is_running_ = false;
  if (connection_monitor_thread_.joinable()) {
    connection_monitor_thread_.join();
  }
}

void RemoteController::lost_connection_monitor() {
  while (is_running_.load()) {
    if (rc_dsm_nanos_since_last_packet() > kCONNECTION_MONITOR_TIMEOUT_NS) {
      spdlog::error(
          "Lost connection to remote controller! Haven't seen in {} nanoseconds. Invoking user-provided callback "
          "function",
          rc_dsm_nanos_since_last_packet());
      packet_loss_callback_();
    }
    std::this_thread::sleep_for(kCONNECTION_MONITOR_LOOP_SLEEP_TIME);
  }
}

}  // namespace flyMS
