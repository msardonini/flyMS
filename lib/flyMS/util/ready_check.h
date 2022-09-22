/**
 * @file ready_check.h
 * @author Mike Sardonini (msardonini@gmail.com)
 * @brief Declaration of ReadyCheck functions
 * @version 0.1
 * @date 2022-09-21
 *
 * @copyright Copyright (c) 2022
 *
 */
#pragma once

namespace flyMS {

/// \brief ,
///
/// \return 0 on success, -1 if the EXITING status was set

/**
 * @brief Blocks execution until either: 1. the start signal is received from the dsm controller, or 2. something calls
 * "rc_set_state(EXITING)" The start signal is three toggles of the kill switch, then left to the ON position
 *
 * @return true if the start signal was received
 * @return false if the EXITING status was set or an error occurred
 */
bool wait_for_start_signal();

}  // namespace flyMS
