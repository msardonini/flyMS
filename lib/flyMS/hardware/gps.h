/**
 * @file gps.hpp
 * @brief Source code to communicate with the trimble copernicus II GPS module. This code is currently deprecated and
 * only here for reference
 *
 * @author Mike Sardonini
 * @date 10/15/2018
 */
#pragma once

// System Includes
#include <fcntl.h>
#include <math.h>
#include <string.h>
#include <strings.h>
#include <termios.h>
#include <unistd.h>

#include <atomic>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

// Package Includes
// #include <roboticscape.h>

#include "yaml-cpp/yaml.h"

namespace flyMS {

#define D2R_GPS 0.01744
#define BAUDRATE B4800
#define BAUDRATE2 B57600
#define MODEMDEVICE "/dev/ttyO1"

typedef struct GPS_data_t {
  int GPS_file;
  float deg_longitude;
  float deg_latitude;
  float gps_altitude;
  float meters_lat;
  float meters_lon;
  float speed;
  float direction;
  double min_longitude;
  double min_latitude;
  float HDOP;
  int GPS_fix;
  double pos_lon, pos_lat;
  int GPS_init_check;
  int GPS_fix_check;
} GPS_data_t;

class gps {
 public:
  gps();

  // Default Destructor
  ~gps();

  void init();

  // Main thread which controls the inner loop FCS
  int flightCore();

  // Initialize the system's hardware
  int startupRoutine();

  int getGpsData(GPS_data_t *_gpsData);

 private:
  int dataMonitor();

  float get_NMEA_field(int field, char buf[], int comma[]);

  void read_raw_gps(char *buf, GPS_data_t *GPS_data);

  GPS_data_t gpsData;

  // Variables to manage the gps threads
  std::thread gpsThread;
  std::timed_mutex gpsMutex;

  // Flags to signal if messages have arrived
  bool GGA_flag;
  bool VTG_flag;
  std::atomic<bool> GPS_data_flag;

  // the serial file descriptor
  int serialFd;
};

}  // namespace flyMS
