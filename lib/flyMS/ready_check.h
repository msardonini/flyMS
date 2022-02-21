#pragma once

/// \brief Blocks execution until either: 1. the start signal is received from the dsm controller, or 2. something calls
/// "rc_set_state(EXITING)" The start signal is three toggles of the kill switch
///
/// \return 0 on success, -1 if the EXITING status was set
int wait_for_start_signal();
