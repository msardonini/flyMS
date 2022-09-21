/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
#include "flyMS/ulog/ulog.h"

#include <cstdint>
#include <iostream>

#include "spdlog/spdlog.h"

namespace flyMS {

constexpr std::size_t kMESSAGE_HEADER_LEN = 3;  //< accounts for msg_size and msg_type

namespace {
typedef enum class ULogMessageType {
  FORMAT = 'F',
  DATA = 'D',
  INFO = 'I',
  INFO_MULTIPLE = 'M',
  PARAMETER = 'P',
  ADD_LOGGED_MSG = 'A',
  REMOVE_LOGGED_MSG = 'R',
  SYNC = 'S',
  DROPOUT = 'O',
  LOGGING = 'L',
  FLAG_BITS = 'B',
} ULogMessageType;

/** first bytes of the file */
struct ulog_file_header_s {
  uint8_t magic[8];
  uint64_t timestamp;
} __attribute__((packed));

struct ulog_message_header_s {
  uint16_t msg_size;
  uint8_t msg_type;
} __attribute__((packed));

struct ulog_message_format_s {
  struct ulog_message_header_s header;
  char format[2096];
} __attribute__((packed));

struct ulog_message_add_logged_s {
  struct ulog_message_header_s header;

  uint8_t multi_id;
  uint16_t msg_id;
  char message_name[255];
} __attribute__((packed));

struct ulog_message_remove_logged_s {
  struct ulog_message_header_s header;

  uint16_t msg_id;
} __attribute__((packed));

struct ulog_message_sync_s {
  struct ulog_message_header_s header;

  uint8_t sync_magic[8];
} __attribute__((packed));

struct ulog_message_dropout_s {
  struct ulog_message_header_s header;

  uint16_t duration;  // in ms
} __attribute__((packed));

struct ulog_message_data_header_s {
  struct ulog_message_header_s header;

  uint16_t msg_id;
} __attribute__((packed));

struct ulog_message_info_header_s {
  struct ulog_message_header_s header;

  uint8_t key_len;
  char key[255];
} __attribute__((packed));

struct ulog_message_info_multiple_header_s {
  struct ulog_message_header_s header;

  uint8_t
      is_continued;  ///< can be used for arrays: set to 1, if this message is part of the previous with the same key
  uint8_t key_len;
  char key[255];
} __attribute__((packed));

struct ulog_message_logging_s {
  struct ulog_message_header_s header;

  uint8_t log_level;  // same levels as in the linux kernel
  uint64_t timestamp;
  char message[128];  // defines the maximum length of a logged message string
} __attribute__((packed));

struct ulog_message_parameter_header_s {
  struct ulog_message_header_s header;

  uint8_t key_len;
  char key[255];
} __attribute__((packed));

struct ulog_message_flag_bits_s {
  struct ulog_message_header_s header;

  uint8_t compat_flags[8];
  uint8_t incompat_flags[8];     ///< @see ULOG_INCOMPAT_FLAG_*
  uint64_t appended_offsets[3];  ///< file offset(s) for appended data if ULOG_INCOMPAT_FLAG0_DATA_APPENDED_MASK is set
} __attribute__((packed));

}  // namespace

ULog::~ULog() {
  is_running_ = false;
  if (write_thread_.joinable()) {
    write_thread_.join();
  }

  fd_.close();
}

std::filesystem::path ULog::generate_intremented_run_dir(const std::filesystem::path &log_folder) {
  auto get_run_str = [](const int run_number) { return fmt::format("run{:03d}", run_number); };

  int run_number = 1;
  std::filesystem::path log_path = log_folder / get_run_str(run_number);
  while (std::filesystem::exists(log_path)) {
    log_path = log_folder / get_run_str(++run_number);
  }
  std::filesystem::create_directory(log_path);

  return log_path;
}

int ULog::init(const std::filesystem::path &log_folder) {
  // Create the log file
  fd_ = std::ofstream(log_folder, std::ios::binary);
  if (!fd_.is_open()) {
    spdlog::error("Error! Could not open ULog file");
    return -1;
  }

  // Writes the header to the file
  write_header();

  // Populate our supported messages map
  msg_formats_.clear();
  msg_formats_.insert(std::make_pair(ULogFlightMsg::ID, std::make_pair(ULogFlightMsg::NAME, ULogFlightMsg::FORMAT)));
  msg_formats_.insert(std::make_pair(ULogGpsMsg::ID, std::make_pair(ULogGpsMsg::NAME, ULogGpsMsg::FORMAT)));
  msg_formats_.insert(
      std::make_pair(ULogPosCntrlMsg::ID, std::make_pair(ULogPosCntrlMsg::NAME, ULogPosCntrlMsg::FORMAT)));

  // Write the file formats to the beginning of the ulog file so the parser knows the data types and formats
  for (const auto &msg_format : msg_formats_) {
    write_formats(msg_format.second.second);
  }

  for (const auto &msg_format : msg_formats_) {
    write_add_log(msg_format.first, msg_format.second.first);
  }

  // Start the write thread
  is_running_ = true;
  write_thread_ = std::thread(&ULog::write_thread, this);

  return 0;
}

void ULog::write_thread() {
  while (is_running_) {
    {
      if (!write_queue_.empty()) {
        std::pair<std::unique_ptr<const char[]>, std::size_t> buf;
        {
          std::scoped_lock lock(queue_mutex_);
          buf = std::move(write_queue_.front());
          write_queue_.pop();
        }
        write_to_file(buf.first.get(), buf.second);
      } else {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    }
  }
}

void ULog::write_msg(const ULogMsg &data) {
  if (!is_running_) {
    spdlog::error("ULog is not running. Call init() before write_flight_data(). Not Logging {} message",
                  data.get_msg_name());
    return;
  }
  if (msg_formats_.find(data.get_msg_id()) == msg_formats_.end()) {
    spdlog::error("ULog does not support {} message. Not Logging it", data.get_msg_name());
    return;
  }

  size_t write_size = sizeof(struct ulog_message_data_header_s) + data.get_msg_size();

  // Populate the header
  ulog_message_header_s header;
  header.msg_size = write_size - kMESSAGE_HEADER_LEN;

  header.msg_type = (char)ULogMessageType::DATA;

  // Populate the header
  struct ulog_message_data_header_s data_header;
  data_header.header = header;
  data_header.msg_id = data.get_msg_id();

  auto msg_size = sizeof(struct ulog_message_data_header_s) + data.get_msg_size();
  auto buf = std::make_unique<char[]>(msg_size);
  // Copy the header and struct
  memcpy(buf.get(), &data_header, sizeof(struct ulog_message_data_header_s));
  memcpy(buf.get() + sizeof(struct ulog_message_data_header_s), data.get_msg_ptr(), data.get_msg_size());

  std::scoped_lock lock(queue_mutex_);
  write_queue_.push(std::make_pair(std::move(buf), msg_size));
}

void ULog::write_add_log(uint16_t id, std::string_view msg_name) {
  struct ulog_message_add_logged_s msg = {};
  size_t write_size = sizeof(struct ulog_message_add_logged_s) - sizeof(msg.message_name) + msg_name.size();

  // Populate the header
  ulog_message_header_s header;
  header.msg_size = write_size - kMESSAGE_HEADER_LEN;
  header.msg_type = (char)ULogMessageType::ADD_LOGGED_MSG;

  // Populate the message
  msg.header = header;
  msg.multi_id = 0x00;
  msg.msg_id = id;

  std::copy(msg_name.begin(), msg_name.end(), msg.message_name);

  write_to_file(&msg, write_size);
}

void ULog::write_formats(std::string_view msg_format) {
  struct ulog_message_format_s msg;
  size_t write_size = sizeof(struct ulog_message_format_s) - sizeof(msg.format) + msg_format.size();

  // Populate the header
  ulog_message_header_s header;
  header.msg_size = write_size - kMESSAGE_HEADER_LEN;
  header.msg_type = (char)ULogMessageType::FORMAT;

  // Populate the message
  msg.header = header;
  std::copy(msg_format.begin(), msg_format.end(), msg.format);

  // Write the message to disk
  write_to_file(&msg, write_size);
}

void ULog::write_header() {
  struct ulog_file_header_s ulog_header = {};
  ulog_header.magic[0] = 'U';
  ulog_header.magic[1] = 'L';
  ulog_header.magic[2] = 'o';
  ulog_header.magic[3] = 'g';
  ulog_header.magic[4] = 0x01;
  ulog_header.magic[5] = 0x12;
  ulog_header.magic[6] = 0x35;
  ulog_header.magic[7] = 0x01;  // file version 1
  ulog_header.timestamp = get_time_us();

  write_to_file(&ulog_header, sizeof(ulog_header));

  // write the Flags message: this MUST be written right after the ulog header
  ulog_message_header_s header;
  header.msg_size = sizeof(struct ulog_message_flag_bits_s) - kMESSAGE_HEADER_LEN;
  header.msg_type = (char)ULogMessageType::FLAG_BITS;

  struct ulog_message_flag_bits_s flag_bits;
  memset(&flag_bits, 0, sizeof(flag_bits));
  flag_bits.header = header;

  write_to_file(&flag_bits, sizeof(flag_bits));
}

uint64_t ULog::get_time_us() {
  struct timespec tv;
  clock_gettime(CLOCK_MONOTONIC, &tv);
  return tv.tv_sec * (uint64_t)1E6 + tv.tv_nsec / (uint64_t)1E3;
}

void ULog::write_to_file(const void *buf, size_t size) {
  fd_.write(reinterpret_cast<const char *>(buf), size);

  // Check if the write failed
  if (std::ofstream::badbit & fd_.rdstate()) {
    spdlog::error("ULog write failed. Error: {}", strerror(errno));
    is_running_ = false;
  }
}

}  // namespace flyMS
