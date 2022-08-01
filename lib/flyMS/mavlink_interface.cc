
#include "flyMS/mavlink_interface.h"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <iostream>

namespace flyMS {

MavlinkInterface::MavlinkInterface(const std::string &serial_dev) : is_running_(false), serial_dev_file_(serial_dev) {
  init();
}

MavlinkInterface::MavlinkInterface(const YAML::Node input_params)
    : MavlinkInterface(input_params["serial_device"].as<std::string>()) {}

int MavlinkInterface::init() {
  // Open the serial port to send messages to the Nano
  serial_dev_ = open(serial_dev_file_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
  if (serial_dev_ == -1) {
    std::cerr << "Failed to open the serial device" << std::endl;
    return -1;
  }

  // Ensire the port is set to NONBLOCK
  fcntl(serial_dev_, F_SETFL, fcntl(serial_dev_, F_GETFL) & ~O_NONBLOCK);

  // Get the current configuration of the serial interface
  struct termios serial_config;
  if (tcgetattr(serial_dev_, &serial_config) < 0) {
    std::cerr << "Failed to get the serial device attributes" << std::endl;
    return -1;
  }

  // Set the serial device configs
  serial_config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);
  serial_config.c_oflag = 0;
  serial_config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
  serial_config.c_cflag &= ~(CSIZE | PARENB);
  serial_config.c_cflag |= CS8;

  // One input byte is enough to return from read() Inter-character timer off
  serial_config.c_cc[VMIN] = 0;
  serial_config.c_cc[VTIME] = 0;

  // Communication speed (simple version, using the predefined constants)
  if (cfsetispeed(&serial_config, B115200) < 0 || cfsetospeed(&serial_config, B115200) < 0) {
    std::cerr << "Failed to set the baudrate on the serial port!" << std::endl;
    return -1;
  }

  // Finally, apply the configuration
  if (tcsetattr(serial_dev_, TCSAFLUSH, &serial_config) < 0) {
    std::cerr << "Failed to set the baudrate on the serial port!" << std::endl;
    return -1;
  }
  serial_read_thread_ = std::thread(&MavlinkInterface::serial_read_thread, this);

  is_running_.store(true);
  return 0;
}

MavlinkInterface::~MavlinkInterface() {
  is_running_.store(false);
  if (serial_read_thread_.joinable()) {
    serial_read_thread_.join();
  }
}

bool MavlinkInterface::get_vio_data(VioData *vio) {
  std::lock_guard<std::mutex> lock(vio_mutex_);
  if (is_new_vio_data_) {
    *vio = vio_;
    is_new_vio_data_ = false;
    return true;
  } else {
    return false;
  }
}

void MavlinkInterface::serial_read_thread() {
  unsigned char buf[1024];
  size_t buf_size = 1024;
  spdlog::info("starting serial read thread");

  while (is_running_.load()) {
    ssize_t ret = read(serial_dev_, buf, buf_size);

    if (ret < 0) {
      spdlog::error("Error on read(), errno: {}", strerror(errno));
    } else if (ret > 0) {
      for (int i = 0; i < ret; i++) {
        mavlink_status_t mav_status;
        mavlink_message_t mav_message;
        uint8_t msg_received = mavlink_parse_char(MAVLINK_COMM_1, buf[i], &mav_message, &mav_status);

        if (msg_received) {
          if (mavlink_message_handlers_.count(mav_message.msgid)) {
            mavlink_message_handlers_[mav_message.msgid](mav_message);
          } else {
            spdlog::error("[MavlinkInterface] Unknown message received: {}", static_cast<uint32_t>(mav_message.msgid));
          }
        }
      }
    } else {
      // Sleep so we don't overload the CPU. This isn't an ideal method, but if use a blocking call on read(), we
      // can't break out of it on the destruction of this object. It will hang forever until bytes are read, which is
      // not always the case
      std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
  }
}

}  // namespace flyMS
