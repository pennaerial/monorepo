#pragma once

#include <uxr/client/client.h>

#include <cstddef>
#include <cstdint>

#include "sensor_msgs/msg/Imu.h"

constexpr uxrQoS_t DEFAULT_QOS = {
  UXR_DURABILITY_VOLATILE,
  UXR_RELIABILITY_BEST_EFFORT,
  UXR_HISTORY_KEEP_LAST,
  1,
};

constexpr uint16_t DEFAULT_PUBLISHER_KEY = 0x01;
constexpr uint16_t DEFAULT_SUBSCRIBER_KEY = 0x01;

using TopicSizeFunc = uint32_t (*)(const void* msg, uint32_t size);
using TopicSerializeFunc = bool (*)(ucdrBuffer* writer, const void* msg);
using TopicDeserializeFunc = bool (*)(ucdrBuffer* reader, void* msg);

inline uint32_t imu_size_of_topic(const void* msg, uint32_t size)
{
  return sensor_msgs_msg_Imu_size_of_topic(static_cast<const sensor_msgs_msg_Imu*>(msg), size);
}

inline bool imu_serialize_topic(ucdrBuffer* writer, const void* msg)
{
  return sensor_msgs_msg_Imu_serialize_topic(writer, static_cast<const sensor_msgs_msg_Imu*>(msg));
}

inline bool imu_deserialize_topic(ucdrBuffer* reader, void* msg)
{
  return sensor_msgs_msg_Imu_deserialize_topic(reader, static_cast<sensor_msgs_msg_Imu*>(msg));
}

/// User-facing topic config. Object IDs are derived from the topic's position
/// in topics[], so users only need to list each topic once.
struct Topic {
  enum class Direction : bool {
    WRITER,
    READER,
  };

  const char* name;
  const char* topic_name;
  const char* type_name;
  // These type hooks must match type_name.
  const uxrQoS_t qos;
  const Direction dir;
  const std::size_t message_size;
  const TopicSizeFunc size_of_topic;
  const TopicSerializeFunc serialize_topic;
  const TopicDeserializeFunc deserialize_topic;
};

constexpr Topic topics[] = {
  {
    .name = "IMU",
    .topic_name = "rt/imu",
    .type_name = "sensor_msgs::msg::dds_::Imu_",
    .qos = DEFAULT_QOS,
    .dir = Topic::Direction::WRITER,
    .message_size = sizeof(sensor_msgs_msg_Imu),
    .size_of_topic = imu_size_of_topic,
    .serialize_topic = imu_serialize_topic,
    .deserialize_topic = imu_deserialize_topic,
  },
};

constexpr std::size_t topic_count = sizeof(topics) / sizeof(topics[0]);

constexpr std::size_t max_topic_message_size()
{
  std::size_t max_size = 0;
  for (const Topic& topic : topics) {
    if (topic.message_size > max_size) {
      max_size = topic.message_size;
    }
  }
  return max_size;
}

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
