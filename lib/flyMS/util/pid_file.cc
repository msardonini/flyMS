#include "flyMS/util/pid_file.h"

#include <signal.h>
#include <unistd.h>

#include <fstream>

#include "spdlog/spdlog.h"

namespace flyMS {

PidFile::PidFile(const std::filesystem::path &pid_file_path) : pid_file_path_(pid_file_path) {
  if (std::filesystem::exists(pid_file_path)) {
    std::ifstream pid_file(pid_file_path);
    int pid;
    pid_file >> pid;
    pid_file.close();

    // If the PID is valid, Check if there is a process running with this PID
    if (pid_file.good() && kill(pid, 0) == 0) {
      throw std::runtime_error("Cannot start Process with pid file " + pid_file_path.filename().string() +
                               ", another instance is already running.");
    } else {
      spdlog::warn("Removing stale PID file for PruManager.");
      if (!std::filesystem::remove(pid_file_path)) {
        throw std::runtime_error("Could not remove stale PID file for PruManager. Do you have permission?");
      }
    }
  }

  // Create the parent directory if it does not already exist
  if (std::filesystem::create_directories(pid_file_path.parent_path())) {
    spdlog::info("Created parent directory for pid file at {}, which did not already exist.",
                 pid_file_path.parent_path().string());
  }

  // Create the PID file
  std::ofstream pid_file(pid_file_path);
  pid_file << getpid();
  pid_file.close();

  if (!pid_file) {
    throw std::runtime_error("Could not create the PID file for PruManager. Do you have permission?");
  }
}

PidFile::~PidFile() { std::filesystem::remove(pid_file_path_); }

}  // namespace flyMS
