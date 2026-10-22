#pragma once

/**
 * @file
 * @brief Picks the encoder backend for this build, so callers can hold encoders by value
 * (no heap allocation) without naming SITL or hardware types directly.
 */

#include "sdkconfig.h"

#if defined(CONFIG_IDF_TARGET_LINUX)
#include "encoder_sitl.hpp"
#else
#include "encoder_hw.hpp"
#endif

namespace drivers
{

#if defined(CONFIG_IDF_TARGET_LINUX)
using EncoderImpl = Encoder_SITL;
#else
using EncoderImpl = Encoder_HW;
#endif

}  // namespace drivers
