#pragma once

#include <cstdint>

namespace payload_controller_config
{

// Linux SITL runs the Micro-XRCE-DDS Agent locally. Hardware uses UART and ignores this value.
inline constexpr char DDS_AGENT_IP[] = "127.0.0.1";
// The payload-controller launch configuration expects its Micro-XRCE-DDS Agent on port 7777.
inline constexpr char DDS_AGENT_PORT[] = "7777";
// Publish the latest IMU sample at 100 Hz, matching the hardware acquisition loop.
inline constexpr uint32_t DDS_PUBLISH_PERIOD_MS = 10;

}  // namespace payload_controller_config
