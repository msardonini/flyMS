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

#include <array>
#include <condition_variable>
#include <mutex>

#include "flyMS/hardware/pru/pru_messages.h"
#include "flyMS/ipc/redis/RedisPublisher.h"
#include "flyMS/ipc/redis/RedisSubscriber.h"
#include "rc/servo.h"
#include "spdlog/spdlog.h"

namespace flyMS {

constexpr auto kPOLL_TIMEOUT =
    std::chrono::seconds(1);  //< The time we wait for a message to be received from the PruManager

/**
 * @brief The PruRequester is a class that helps handle the ownership of the programmable real-time unit (PRU) on the
 * BeagleBone, which gives commands to ESCs/motors. The PruRequester communicates with the PruManager to gain access to
 * the PRU hardware. If access is granted, the PruRequester is able to send commands to the PRU. The member functions
 * that can be used to interface with this object are:
 * - PruRequester::request_access() - Requests access to the PRU from the PruManager
 * - PruRequester::release_access() - Releases access to the PRU, notify the PruManager
 * - PruRequester::send_command() - Sends a command to the PRU
 *
 *  Example usage is:
 * ```
 * {
 *   PruRequester pru_requester;
 *   if (pru_requester.request_access()) {
 *     std::array<float, 4> motor_commands{0.0, 0.0, 0.0, 0.0};
 *     pru_requester.send_command(motor_commands);
 *   }
 * } // release access to the PRU on destruction
 * ```

 * The messages this object uses to communicate with the PruManager are defined in pru_messages.h
 */
class PruRequester {
 public:
  /**
   * @brief Construct a new Pru Requester object. Initializes the RedisSubscriber and RedisPublisher objects used for
   * inter process communication
   *
   */
  PruRequester()
      : redis_subscriber_(std::bind(&PruRequester::redis_callback, this, std::placeholders::_1, std::placeholders::_2),
                          {kredis_request_response_channel, kredis_release_response_channel},
                          [](const sw::redis::Error& er) {}) {}

  /**
   * @brief Destroy the Pru Requester object. If the PruRequester has PRU ownship at this time, it will release access
   * here
   *
   */
  ~PruRequester() {
    if (has_ownership_of_pru_) {
      release_access();
    }
  }

  /**
   * @brief Request access to the PRU from the PruManager. If access is granted, the PruRequester will be able to send
   * commands to the PRU via the send_command() member function.
   *
   * @return true Access to the PRU was granted
   * @return false Access to the PRU was denied
   */
  bool request_access() {
    PruRequest request{.pid = getpid()};
    redis_publisher_.publish(kredis_request_channel, to_yaml(request));
    std::unique_lock<std::mutex> lock(cv_mutex_);
    if (!cv_.wait_for(lock, kPOLL_TIMEOUT, [this]() { return response_received_; })) {
      throw std::runtime_error("Timeout waiting for response from PRU. Is the PruManager running?");
    }
    response_received_ = false;

    if (response_.success) {
      has_ownership_of_pru_ = true;
      rc_servo_init();
      return true;
    } else {
      spdlog::error("Could not get access to PRU. Reason: {}", response_.reason);
      return false;
    }
  }

  /**
   * @brief Release access to the PRU. This will notify the PruManager that the PruRequester no longer needs access to
   * the PRU.
   *
   * @return true Release was successful
   * @return false Release was unsuccessful
   */
  bool release_access() {
    rc_servo_cleanup();
    has_ownership_of_pru_ = false;

    PruRequest request{.pid = getpid()};
    redis_publisher_.publish(kredis_release_channel, to_yaml(request));
    std::unique_lock<std::mutex> lock(cv_mutex_);
    if (!cv_.wait_for(lock, kPOLL_TIMEOUT, [this]() { return response_received_; })) {
      throw std::runtime_error("Timeout waiting for response from PRU. Is the PruManager running?");
    }
    response_received_ = false;

    if (response_.success) {
      return true;
    } else {
      throw std::runtime_error("Could not release access to PRU. Reason: " + response_.reason);
    }
  }

  /**
   * @brief Send a command to the PRU. This will only work if the PruRequester has access to the PRU.
   *
   * @param command The command to send to the PRU
   */
  void send_command(std::array<float, 4>& commands) {
    if (!has_ownership_of_pru_) {
      throw std::runtime_error("Cannot send command to PRU. Do not have ownership of PRU.");
    }
    for (auto chan = 0ul; chan < commands.size(); ++chan) {
      rc_servo_send_esc_pulse_normalized(chan, commands[chan]);
    }
  }

 private:
  /**
   * @brief Callback function for the RedisSubscriber for when messages are received
   *
   * @param channel The channel the message was received on
   * @param message The message that was received
   */
  void redis_callback(std::string_view channel, std::string_view message) {
    // TODO handle channels differently
    if (channel == std::string(kredis_request_response_channel)) {
      spdlog::info("Received request response");
      response_ = from_yaml<PruResponse>(YAML::Load(message.data()));
      response_received_ = true;
      cv_.notify_one();
    } else if (channel == std::string(kredis_release_response_channel)) {
      spdlog::info("Received release response");
      response_ = from_yaml<PruResponse>(YAML::Load(message.data()));
      spdlog::info("{}", response_.reason);
      response_received_ = true;
      cv_.notify_one();
    } else {
      spdlog::error("Received message on unknown channel: {}", channel);
    }
  }

  RedisSubscriber redis_subscriber_;  //< For receiving messages from the PruManager
  RedisPublisher redis_publisher_;    //< For sending messages to the PruManager
  PruResponse response_;              //< The response received from the PruManager

  std::mutex cv_mutex_;                //< Mutex for the condition variable
  std::condition_variable cv_;         //< For synchronizing requests/responses
  bool response_received_ = false;     //< has a response been received from the PruManager
  bool has_ownership_of_pru_ = false;  //< Flag for PRU ownership
};

}  // namespace flyMS
