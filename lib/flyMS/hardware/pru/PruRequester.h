/**
 * @file PruRequester.h
 * @author Mike Sardonini
 * @brief PruRequester is a class which handles the ownership and use of the programmable real-time unit (PRU) on the
 * BeagleBone, which is used to give commands to ESCs/motors.
 * @version 0.1
 * @date 2022-09-17
 *
 * @copyright Copyright (c) 2022
 *
 */
#pragma once

#include <sys/file.h>
#include <unistd.h>

#include <cerrno>
#include <filesystem>
#include <fstream>

#include "rc/servo.h"
#include "spdlog/spdlog.h"

namespace flyMS {

constexpr char kPRU_LOCK_FILE[] = "/tmp/pru_lock_file";

/**
 * @brief The PruRequester is a class that helps handle the ownership of the programmable real-time unit (PRU) on the
 * BeagleBone, which gives commands to ESCs/motors. It is critical that only one process is able to commands to the PRU
 at a time. Thus, this object creates a lockfile saved at 'kPRU_LOCK_FILE' to ensure that only one process can access
 the PRU at a time. On construction, the object will spawn a thread that continuously sends zero commands to the PRU.
 This prevents ESC's from 'chirping' due to not receiving commands. The zero sender thread will run until 'send_command'
 is called, at which point it will stop the zero sender thread and send the new command. The zero sender thread can be
 restarted by calling 'start_zero_sender'.
 * - PruRequester::PruRequestor() - Construct the object and lock the PRU. Throw an exception if the lock fails.
 * - PruRequester::send_command() - Sends a command to the PRU
 *
 *  Example usage is:
 * ```
 * {
 *   PruRequester pru_requester;
 *   // The motors will be receiving zero commands until send_command is called
 *   while (in_control_loop) {
 *     std::array<float, 4> motor_commands{0.0, 0.0, 0.0, 0.0};
 *     if(!pru_requester.send_command(motor_commands)) {
 *       // handle error
 *     }
 *   }
 *  // Start the zero_sender thread again now the control loop is done
 *  pru_requester.start_zero_sender();
 * } // release the pru lock file on destruction
 * ```
 *
 */
class PruRequester {
 public:
  /**
   * @brief Construct a new Pru Requester object
   */
  PruRequester() {
    lock_file_ = open(kPRU_LOCK_FILE, O_CREAT | O_RDWR, 0666);
    if (lock_file_ == -1) {
      throw std::runtime_error("Failed to open the PRU lock file!");
    }

    if (flock(lock_file_, LOCK_EX | LOCK_NB) == -1) {
      if (errno == EWOULDBLOCK) {
        throw std::runtime_error("Failed to lock the PRU lock file, another process currently holds the lock!");
      } else {
        throw std::runtime_error("Failed to lock the PRU lock file!");
      }
    }

    if (rc_servo_init()) {
      throw std::runtime_error("Failed to initialize the PRU/ESC!");
    }

    start_zero_sender();
  }

  ~PruRequester() {
    zero_sender_running_ = false;
    if (zero_sender_thread.joinable()) {
      zero_sender_thread.join();
    }
    flock(lock_file_, LOCK_UN);
  }

  /**
   * @brief Sends commands to ESCs, and stops the zero sender thread if it is running
   *
   * @param motor_commands commands in the range [0, 1] to send to the ESCs
   * @return true on success
   * @return false on failure
   */
  bool send_command(const std::vector<float>& motor_commands) {
    if (motor_commands.size() > RC_SERVO_CH_MAX) {
      spdlog::error("Too many motor commands! Max is {}", RC_SERVO_CH_MAX);
      return false;
    }

    // If the zero sender is running, stop it
    if (zero_sender_running_) {
      zero_sender_running_ = false;
      if (zero_sender_thread.joinable()) {
        zero_sender_thread.join();
      }
    }

    int channel = 1;
    for (const auto& cmd : motor_commands) {
      if (rc_servo_send_esc_pulse_normalized(channel++, cmd)) {
        spdlog::error("Failed to send command to the PRU/ESC!");
        return false;
      }
    }
    return true;
  }

  bool start_zero_sender() {
    if (zero_sender_running_) {
      spdlog::error("Zero sender is already running!");
      return false;
    }
    zero_sender_running_ = true;
    zero_sender_thread = std::thread(&PruRequester::zero_sender, this);
    return true;
  }

 private:
  void zero_sender() {
    std::vector<float> zero_command(RC_SERVO_CH_MAX);
    for (auto& cmd : zero_command) {
      cmd = 0.0;
    }

    spdlog::info("Starting zero sender thread");
    while (zero_sender_running_) {
      int channel = 1;
      for (const auto& cmd : zero_command) {
        if (rc_servo_send_esc_pulse_normalized(channel++, cmd)) {
          spdlog::error("Failed to send command to the PRU/ESC!");
          return;
        }
      }

      // Run at 200 Hz
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }

  std::thread zero_sender_thread;
  std::atomic<bool> zero_sender_running_{false};
  int lock_file_ = -1;  //< File to lock access to the PRU
};

}  // namespace flyMS
