#pragma once

#include <rosidl_runtime_cpp/traits.hpp>

namespace pennair_vision
{

template <typename T>
concept RosMessage = rosidl_generator_traits::is_message<T>::value;

}  // namespace pennair_vision
