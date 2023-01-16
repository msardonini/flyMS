/**
 * @file ready_check.cc
 * @author Mike Sardonini (msardonini@gmail.com)
 * @brief Definition of ReadyCheck functions
 * @version 0.1
 * @date 2022-09-21
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "flyMS/util/ready_check.h"

#include <chrono>
#include <iostream>
#include <mutex>
#include <thread>

#include "flyMS/common/constants.h"
#include "flyMS/hardware/RemoteController.h"
#include "rc/led.h"
#include "rc/start_stop.h"
#include "spdlog/spdlog.h"

namespace flyMS {

constexpr uint32_t READY_CHECK_LOOP_FREQUENCY_HZ = 100;
constexpr auto READY_CHECK_SLEEP_TIME = std::chrono::microseconds(1000000 / READY_CHECK_LOOP_FREQUENCY_HZ);
constexpr uint32_t BLINK_FREQUENCY_HZ = 5;
constexpr uint32_t MAX_TOGGLE_COUNT = 6;
constexpr float SWITCH_THRESHOLD_VAL = 0.25f;  // dsm switch vals come in as floats. Need thresh to detect switches

bool wait_for_start_signal() {
  // Toggle the kill switch to get going, to ensure controlled take-off
  // Keep kill switch down to remain operational
  std::size_t toggle_count = 0;
  std::size_t loop_count = 0;
  int green_led_value = 0;
  float prev_switch_val = 0.0f;
  std::once_flag first_run;
  rc_led_set(RC_LED_RED, 0);
  rc_led_set(RC_LED_GREEN, 0);
  spdlog::info("Toggle the kill switch twice and leave up to initialize");
  while (rc_get_state() != EXITING && toggle_count < MAX_TOGGLE_COUNT) {
    // Blink the green LED light to signal that the program is ready
    if (loop_count++ % (READY_CHECK_LOOP_FREQUENCY_HZ / BLINK_FREQUENCY_HZ) == 0) {
      green_led_value = (green_led_value ? 0 : 1);
      rc_led_set(RC_LED_GREEN, green_led_value);
    }

    std::this_thread::sleep_for(READY_CHECK_SLEEP_TIME);  // Run at 100 Hz

    auto rc_channel_data = RemoteController::get_instance().get_channel_data();
    if (rc_channel_data.empty()) {
      continue;
    }
    std::call_once(first_run, [&]() { prev_switch_val = rc_channel_data[kFLYMS_KILL_SWITCH_INDEX]; });
    float switch_val = rc_channel_data[kFLYMS_KILL_SWITCH_INDEX];

    if (std::abs(switch_val - prev_switch_val) > SWITCH_THRESHOLD_VAL) {
      toggle_count++;
    }

    prev_switch_val = switch_val;
  }

  // Make sure the kill switch disengaged before starting
  while (rc_get_state() != EXITING &&
         RemoteController::get_instance().get_channel_data()[kFLYMS_KILL_SWITCH_INDEX] < SWITCH_THRESHOLD_VAL) {
    std::this_thread::sleep_for(READY_CHECK_SLEEP_TIME);
  }

  if (rc_get_state() != EXITING) {
    spdlog::info("Initialized! Starting program");
    rc_led_set(RC_LED_GREEN, 1);
    rc_led_set(RC_LED_RED, 0);
  } else {
    spdlog::warn("Received shutdown signal during ready check!");
    return false;
  }

  return true;
}

}  // namespace flyMS
