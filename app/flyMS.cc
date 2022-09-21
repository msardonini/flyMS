/**
 * @file flyMS.cpp
 * @brief Application entry point of the flyMS program.
 *
 * @author Mike Sardonini
 * @date 10/15/2018
 */

#include <getopt.h>
#include <signal.h>

#include <chrono>
#include <memory>
#include <string>

#include "flyMS/FlightCore.h"
#include "flyMS/config_requestor.h"
#include "flyMS/debug_mode.hpp"
#include "flyMS/ready_check.h"
#include "rc/start_stop.h"
#include "spdlog/spdlog.h"
#include "yaml-cpp/yaml.h"

/**
 * @brief Callback function for signals. SIGHUP (hangup signal) is ignored, others will stop the program
 *
 * @param signo The signal number received
 */
static void on_signal_received(int signo) {
  switch (signo) {
    case SIGHUP:
      break;
    default:
      rc_set_state(EXITING);
  }
}

void init_signal_handler() {
  signal(SIGINT, on_signal_received);
  signal(SIGKILL, on_signal_received);
  signal(SIGTERM, on_signal_received);
  signal(SIGHUP, on_signal_received);
}

/**
 * @brief Shutdown the program and signal on the LEDs there was an error
 *
 */
void shutdown_fail() {
  rc_set_state(EXITING);
  rc_led_set(RC_LED_GREEN, 0);
  rc_led_set(RC_LED_RED, 1);
}

/**
 * @brief Shutdown the program and turn off the LEDs
 *
 */
void shutdown_success() {
  rc_set_state(EXITING);
  rc_led_set(RC_LED_GREEN, 0);
  rc_led_set(RC_LED_RED, 0);
}

/**
 * @brief Application entry point of the flyMS program
 */
int main() {
  init_signal_handler();

  try {
    if constexpr (flyMS::kDEBUG_MODE) {
      if (!flyMS::wait_for_start_signal()) {
        shutdown_fail();
        return -1;
      }
    }

    // Load the Yaml Node
    YAML::Node config_params = flyMS::get_config("http://localhost:5001/config");
    if (config_params.size() == 0) {
      shutdown_fail();
      return -1;
    } else {
      config_params = config_params["flyMSParams"];
    }

    rc_set_state(UNINITIALIZED);

    flyMS::FlightCore fly(config_params);
    // Initialize the flight hardware
    if (!fly.init()) {
      rc_set_state(EXITING);
      spdlog::error("Startup failed, exiting");
      shutdown_fail();
      return -1;
    } else {
      rc_led_set(RC_LED_GREEN, 1);
      rc_led_set(RC_LED_RED, 0);
    }
    while (rc_get_state() != EXITING) {
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }

  } catch (const std::exception &e) {
    spdlog::error("Exception caught in main: {}", e.what());
    shutdown_fail();
    return -1;
  } catch (...) {
    spdlog::error("Unknown exception caught in main");
    shutdown_fail();
    return -1;
  }
  shutdown_success();
  return 0;
}
