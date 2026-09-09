#pragma once

#include <cstdint>
#include <uxr/client/client.h>

// TODO: write more pub/subs here
enum class Topic : uint8_t {
  // publishers
  IMU_PUB,

  // subscribers
  CMD_VEL_SUB,
};

/// Converts Topic enum to underlying uint8_t type
inline constexpr uint8_t to_underlying(const Topic topic)
{
  return static_cast<uint8_t>(topic);
}

/// groups publisher related fields together
struct Publisher {
  const uint8_t topic_id;
  const uint8_t pub_id;
  const uxrObjectId dw_id; // datawriter
  const char* topic_name;
  const char* type_name;
  const uxrQoS_t qos;
};


/// groups subscriber related fields together
struct Subscriber {
  const uint8_t topic_id;
  const uint8_t sub_id;
  const uxrObjectId dr_id; // datareader
  const char* topic_name;
  const char* type_name;
  const uxrQoS_t qos;
};

// TODO: fill this out
constexpr Publisher publishers[] =
{


};

// TODO: fill this out
constexpr Subscriber subscribers[] =
{

};
