/**
 * @file pruHandler.hpp
 * @brief Process which communicates with the PRU that controlls quadrotor ESCs.
     This process connects to applications via TCP packets over localhost and
     relays the data to the PRU. If no program is active, zero throttle is sent.
 *
 * @author Mike Sardonini
 * @date 11/10/2018
 */
#pragma once

// System Includes
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

#include <fstream>
#include <thread>

// #include "rc/pthread.h"
#include "rc/servo.h"
#include "rc/time.h"

enum class PruState { UNINITIALIZED, RUNNING, PAUSED, EXITING };

class pruHandler {
 public:
  // Default Constructor
  pruHandler();

  // Default Destructor
  ~pruHandler();

  void shutdownPruHandler(int signo);
  int init_pru_handler();

 private:
  int pru_set_state(enum PruState new_state);
  enum PruState pru_get_state();

  int checkForCompetingProcess();
  int createPIDFile();
  int initServer();
  int run();

  // State of the Program
  enum PruState pruState;

  std::thread pruHandlerThread;

  int listenfd;
  int connfd;
  int n = 0;
  struct sockaddr_in serv_addr;

  char rcvBuff[16];

  std::ofstream logFid;
};
