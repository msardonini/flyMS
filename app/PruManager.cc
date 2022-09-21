/**
 * @file PruManager.cc
 * @author Mike Sardonini (msardonini@gmail.com)
 * @brief Very simple application with instantiates a PruManager object and runs, and runs until the interrupt signal is
 * received
 * @version 0.1
 * @date 2022-09-17
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "flyMS/hardware/pru/PruManager.h"

#include <thread>

bool is_running = true;

static void on_signal_received(int signo) {
  if (signo == SIGHUP) {
    return;
  }
  is_running = false;
}

void init_signal_handler() {
  signal(SIGINT, on_signal_received);
  signal(SIGKILL, on_signal_received);
  signal(SIGHUP, on_signal_received);
}

int main() {
  init_signal_handler();
  flyMS::PruManager zero_sender;

  while (is_running) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}
