/// \file Defines the global variable kDEBUG_MODE that enables/disables debugging features
#pragma once

#ifdef DEBUG
static constexpr bool kDEBUG_MODE = true;
#else
static constexpr bool kDEBUG_MODE = false;
#endif