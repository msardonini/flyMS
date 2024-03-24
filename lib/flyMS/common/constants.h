#pragma once

#include <cstdint>

namespace flyMS {

constexpr uint32_t kFLYMS_CONTROL_LOOP_FREQUENCY = 200;                         //< Frequency of the control loop
constexpr double kFLYMS_CONTROL_LOOP_DT = 1.0 / kFLYMS_CONTROL_LOOP_FREQUENCY;  //< Delta t for the control loop

// Verbose names for the physical channels of the remote controller
static constexpr uint32_t kFLYMS_THROTTLE_INDEX = 0;
static constexpr uint32_t kFLYMS_ROLL_INDEX = 1;
static constexpr uint32_t kFLYMS_PITCH_INDEX = 2;
static constexpr uint32_t kFLYMS_YAW_INDEX = 3;
static constexpr uint32_t kFLYMS_KILL_SWITCH_INDEX = 4;
static constexpr uint32_t kFLYMS_FLIGHT_MODE_INDEX = 5;

}  // namespace flyMS
