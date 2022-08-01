
#include "flyMS/ready_check.h"

#include <chrono>
#include <iostream>
#include <thread>

#include "rc/dsm.h"
#include "rc/led.h"
#include "rc/start_stop.h"
#include "spdlog/spdlog.h"

namespace flyMS {

constexpr uint32_t READY_CHECK_LOOP_FREQUENCY_HZ = 100;
constexpr uint32_t READY_CHECK_SLEEP_TIME_US = 1E6 / READY_CHECK_LOOP_FREQUENCY_HZ;
constexpr uint32_t BLINK_FREQUENCY_HZ = 5;
constexpr uint32_t MAX_TOGGLE_COUNT = 6;
constexpr float SWITCH_THRESHOLD_VAL = 0.25f;  // dsm switch vals come in as floats. Need thresh to detect switches
constexpr uint32_t SWITCH_CHANNEL = 5;

int wait_for_start_signal() {
  // Initialze the serial RC hardware
  if (rc_dsm_init()) {
    return -1;
  }

  // Toggle the kill switch to get going, to ensure controlled take-off
  // Keep kill switch down to remain operational
  int led_status = 0;
  std::size_t toggle_count = 0;
  std::size_t loop_count = 0;
  float prev_switch_val = 0.0f;
  bool first_run = true;
  rc_led_set(RC_LED_RED, 0);
  rc_led_set(RC_LED_GREEN, 0);
  std::cout << "Toggle the kill swtich twice and leave up to initialize" << std::endl;
  while (toggle_count < MAX_TOGGLE_COUNT && rc_get_state() != EXITING) {
    // Blink the green LED light to signal that the program is ready
    if (loop_count % (READY_CHECK_LOOP_FREQUENCY_HZ / BLINK_FREQUENCY_HZ) == 0) {
      led_status = (led_status) ? 0 : 1;
      rc_led_set(RC_LED_GREEN, led_status);
    }

    if (rc_dsm_is_new_data()) {
      // Skip the first run to let data history fill up
      if (first_run) {
        prev_switch_val = rc_dsm_ch_normalized(SWITCH_CHANNEL);
        first_run = false;
        continue;
      }

      float switch_val = rc_dsm_ch_normalized(SWITCH_CHANNEL);
      if (std::abs(switch_val - prev_switch_val) > SWITCH_THRESHOLD_VAL) {
        toggle_count++;
      }
      prev_switch_val = switch_val;
    }
    std::this_thread::sleep_for(std::chrono::microseconds(READY_CHECK_SLEEP_TIME_US));  // Run at 100 Hz
    loop_count++;
  }

  // Make sure the kill switch disengaged before starting
  float switch_val = rc_dsm_ch_normalized(SWITCH_CHANNEL);
  while (switch_val < SWITCH_THRESHOLD_VAL && rc_get_state() != EXITING) {
    if (rc_dsm_is_new_data()) {
      switch_val = rc_dsm_ch_normalized(SWITCH_CHANNEL);
    }
    std::this_thread::sleep_for(std::chrono::microseconds(READY_CHECK_SLEEP_TIME_US));
  }

  if (rc_get_state() != EXITING) {
    spdlog::info("Initialized! Starting program");
    rc_led_set(RC_LED_GREEN, 1);
    rc_led_set(RC_LED_RED, 0);
  } else {
    spdlog::warn("Received shutdown signal during ready check!");
    rc_led_set(RC_LED_GREEN, 0);
    rc_led_set(RC_LED_RED, 1);
    return -1;
  }

  rc_dsm_cleanup();
  return 0;
}

}  // namespace flyMS
