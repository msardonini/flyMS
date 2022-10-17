#pragma once

#include <cstdint>

constexpr uint32_t kFLYMS_CONTROL_LOOP_FREQUENCY = 200;                         //< Frequency of the control loop
constexpr double kFLYMS_CONTROL_LOOP_DT = 1.0 / kFLYMS_CONTROL_LOOP_FREQUENCY;  //< Delta t for the control loop
