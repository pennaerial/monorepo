#pragma once

#include <uxr/client/client.h>

#include <cstddef>
#include <cstdint>

constexpr uxrQoS_t DEFAULT_QOS = {
  UXR_DURABILITY_VOLATILE,
  UXR_RELIABILITY_BEST_EFFORT,
  UXR_HISTORY_KEEP_LAST,
  1,
};

constexpr uint16_t DEFAULT_PUBLISHER_KEY = 0x01;
constexpr uint16_t DEFAULT_SUBSCRIBER_KEY = 0x01;

/// User-facing topic config. Object IDs are derived from the topic's position
/// in topics[], so users only need to list each topic once.
struct Topic {
  enum class Direction : bool {
    WRITER,
    READER
  };
  
  const char* name;
  const char* topic_name;
  const char* type_name;
  const uxrQoS_t qos;
  const Direction dir;
};

constexpr Topic topics[] = {
  {
    .name = "IMU",
    .topic_name = "rt/imu",
    .type_name = "sensor_msgs::msg::dds_::Imu_",
    .qos = DEFAULT_QOS,
    .dir = Topic::Direction::WRITER
  },
};

constexpr std::size_t topic_count = sizeof(topics) / sizeof(topics[0]);
constexpr std::size_t IMU_TOPIC_INDEX = 0;

constexpr std::size_t datawriter_count()
{
  std::size_t count = 0;
  for (const Topic& topic : topics) {
    if (topic.dir == Topic::Direction::WRITER) {
      ++count;
    }
  }
  return count;
}

constexpr std::size_t datareader_count()
{
  std::size_t count = 0;
  for (const Topic& topic : topics) {
    if (topic.dir == Topic::Direction::READER) {
      ++count;
    }
  }
  return count;
}

constexpr uint16_t object_key(const std::size_t topic_index)
{
  return static_cast<uint16_t>(topic_index + 1);
}

inline uxrObjectId topic_id(const std::size_t topic_index)
{
  return uxr_object_id(object_key(topic_index), UXR_TOPIC_ID);
}

inline uxrObjectId datawriter_id(const std::size_t topic_index)
{
  return uxr_object_id(object_key(topic_index), UXR_DATAWRITER_ID);
}

inline uxrObjectId datareader_id(const std::size_t topic_index)
{
  return uxr_object_id(object_key(topic_index), UXR_DATAREADER_ID);
}
