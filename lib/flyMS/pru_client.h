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

// Package Includes
#include "roboticscape.h"

namespace flyMS {

class pruClient {
 public:
  // Default Constructor
  pruClient();

  // Default Destructor
  ~pruClient();

  int setSendData(std::array<float, 4> u);

  int startPruClient();

 private:
  // Socket variables
  int sockfd_;
};

}  // namespace flyMS
