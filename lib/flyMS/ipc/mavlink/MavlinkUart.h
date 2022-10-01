#pragma once

#include <atomic>
#include <filesystem>
#include <thread>

#include "flyMS/ipc/mavlink/MavlinkParser.h"
#include "flyMS/ipc/uart/Uart.h"

namespace flyMS {

/**
 * @brief
 *
 */
class MavlinkUart : public MavlinkParser, private Uart {
 public:
  /**
   * @brief Construct a new Redis Interface object
   */
  MavlinkUart(const std::filesystem::path& serial_dev)
      : Uart(serial_dev, std::bind(&MavlinkUart::receive_bytes, this, std::placeholders::_1, std::placeholders::_2)) {}

  template <typename MavType>
  void send_mavlink_msg(MavType& msg,
                        std::function<uint16_t(uint8_t, uint8_t, mavlink_message_t*, const MavType*)> encode_func) {
    mavlink_message_t mav_message;
    auto size = encode_func(0, 0, &mav_message, &msg);
    std::vector<uint8_t> bytes(size);
    mavlink_msg_to_send_buffer(bytes.data(), &mav_message);
    send(bytes);
  }

 private:
  void receive_bytes(const std::array<uint8_t, kBUF_SIZE>& bytes, size_t size);

  std::atomic<bool> is_running_;
  int serial_fd_;                   //< file descriptor for the serial port
  std::thread serial_read_thread_;  //< Thread that reads the serial port for new data

  bool init(const std::filesystem::path& serial_dev);

  void serial_read_thread();
};

}  // namespace flyMS
