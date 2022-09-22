/**
 * @file debug_mode.h
 * @author Mike Sardonini (msardonini@gmail.com)
 * @brief Defines the global variable kDEBUG_MODE that enables/disables debugging features
 * @version 0.1
 * @date 2022-09-21
 *
 * @copyright Copyright (c) 2022
 *
 */
#pragma once

namespace flyMS {

/**
 * @brief Defines the global constexpr variable kDEBUG_MODE that enables/disables debugging features. If used in an `if
 * constexpr` statement, the compiler will optimize out the code block if kDEBUG_MODE is false leaving no runtime
 * overhead in the production binary
 *
 */
#ifdef DEBUG
static constexpr bool kDEBUG_MODE = true;
#else
static constexpr bool kDEBUG_MODE = false;
#endif

}  // namespace flyMS
