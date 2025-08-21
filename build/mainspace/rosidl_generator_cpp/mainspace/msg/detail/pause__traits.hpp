// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mainspace:msg/Pause.idl
// generated code does not contain a copyright notice

#ifndef MAINSPACE__MSG__DETAIL__PAUSE__TRAITS_HPP_
#define MAINSPACE__MSG__DETAIL__PAUSE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mainspace/msg/detail/pause__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace mainspace
{

namespace msg
{

inline void to_flow_style_yaml(
  const Pause & msg,
  std::ostream & out)
{
  out << "{";
  // member: pause
  {
    out << "pause: ";
    rosidl_generator_traits::value_to_yaml(msg.pause, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Pause & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: pause
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pause: ";
    rosidl_generator_traits::value_to_yaml(msg.pause, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Pause & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace mainspace

namespace rosidl_generator_traits
{

[[deprecated("use mainspace::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const mainspace::msg::Pause & msg,
  std::ostream & out, size_t indentation = 0)
{
  mainspace::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mainspace::msg::to_yaml() instead")]]
inline std::string to_yaml(const mainspace::msg::Pause & msg)
{
  return mainspace::msg::to_yaml(msg);
}

template<>
inline const char * data_type<mainspace::msg::Pause>()
{
  return "mainspace::msg::Pause";
}

template<>
inline const char * name<mainspace::msg::Pause>()
{
  return "mainspace/msg/Pause";
}

template<>
struct has_fixed_size<mainspace::msg::Pause>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<mainspace::msg::Pause>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<mainspace::msg::Pause>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MAINSPACE__MSG__DETAIL__PAUSE__TRAITS_HPP_
