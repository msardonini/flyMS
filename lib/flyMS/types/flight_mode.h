/**
 * @file flight_mode.h
 * @author Mike Sardonini (msardonini@gmail.com)
 * @brief Declaration of FlightMode enum
 * @version 0.1
 * @date 2022-09-21
 *
 * @copyright Copyright (c) 2022
 *
 */

#pragma once

namespace flyMS {
enum class FlightMode : uint32_t { UNDEFINED = 0, STABILIZED = 1, ACRO = 2 };
}  // namespace flyMS
