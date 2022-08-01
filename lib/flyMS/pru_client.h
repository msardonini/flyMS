/**
 * @file flyMS.cpp
 * @brief flyMS program source code.
 *
 * @author Mike Sardonini
 * @date 10/15/2018
 */

#pragma once

// System Includes
#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

// C++ includes
#include <iostream>
#include <mutex>
#include <thread>
#include <vector>

namespace flyMS {

/**
 * @brief Client which handles communications with the pru_handler daemon to send commands to Electronic Speed
 * Controllers (ESCs)
 *
 */
class PruClient {
 public:
  // Default Constructor
  PruClient();

  // Default Destructor
  ~PruClient();

  /**
   * @brief Sends a command to the pru_handler daemon to set the ESCs to a specific speed
   *
   * @param u Array of values to send to the ESCs, motors 1-4
   */
  void send_data(std::array<float, 4> u);

  /**
   * @brief Initializes the pru_handler
   *
   * @return int 0 on success, -1 on failure
   */
  int init();

 private:
  // Socket variables
  int sockfd_;
};

}  // namespace flyMS
