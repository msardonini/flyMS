#pragma once

#include <filesystem>

namespace flyMS {

/**
 * @brief Creates a PID file at the given path and ensures that no other process has such a file. The file is deleted
 * upon object destruction. If a file exists with no corresponding process (by its PID), the file will be destroyed. If
 * another process is running with the same PID, or there is an issue saving the file, a std::runtime_error will be
 * thrown.
 *
 */
class PidFile {
 public:
  /**
   * @brief Create the pid file at the given path
   *
   * @param pid_file Path to the PID file to create.
   */
  PidFile(const std::filesystem::path &pid_file_path);

  /**
   * @brief Destroy the Pid File object. The pid file will be deleted on destruction.
   *
   */
  ~PidFile();

 private:
  std::filesystem::path pid_file_path_;
};

}  // namespace flyMS
