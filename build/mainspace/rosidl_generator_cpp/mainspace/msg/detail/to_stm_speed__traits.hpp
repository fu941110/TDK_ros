// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mainspace:msg/ToStmSpeed.idl
// generated code does not contain a copyright notice

#ifndef MAINSPACE__MSG__DETAIL__TO_STM_SPEED__TRAITS_HPP_
#define MAINSPACE__MSG__DETAIL__TO_STM_SPEED__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mainspace/msg/detail/to_stm_speed__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace mainspace
{

namespace msg
{

inline void to_flow_style_yaml(
  const ToStmSpeed & msg,
  std::ostream & out)
{
  out << "{";
  // member: vx
  {
    out << "vx: ";
    rosidl_generator_traits::value_to_yaml(msg.vx, out);
    out << ", ";
  }

  // member: vy
  {
    out << "vy: ";
    rosidl_generator_traits::value_to_yaml(msg.vy, out);
    out << ", ";
  }

  // member: w
  {
    out << "w: ";
    rosidl_generator_traits::value_to_yaml(msg.w, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ToStmSpeed & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: vx
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vx: ";
    rosidl_generator_traits::value_to_yaml(msg.vx, out);
    out << "\n";
  }

  // member: vy
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vy: ";
    rosidl_generator_traits::value_to_yaml(msg.vy, out);
    out << "\n";
  }

  // member: w
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "w: ";
    rosidl_generator_traits::value_to_yaml(msg.w, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ToStmSpeed & msg, bool use_flow_style = false)
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
  const mainspace::msg::ToStmSpeed & msg,
  std::ostream & out, size_t indentation = 0)
{
  mainspace::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mainspace::msg::to_yaml() instead")]]
inline std::string to_yaml(const mainspace::msg::ToStmSpeed & msg)
{
  return mainspace::msg::to_yaml(msg);
}

template<>
inline const char * data_type<mainspace::msg::ToStmSpeed>()
{
  return "mainspace::msg::ToStmSpeed";
}

template<>
inline const char * name<mainspace::msg::ToStmSpeed>()
{
  return "mainspace/msg/ToStmSpeed";
}

template<>
struct has_fixed_size<mainspace::msg::ToStmSpeed>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<mainspace::msg::ToStmSpeed>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<mainspace::msg::ToStmSpeed>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MAINSPACE__MSG__DETAIL__TO_STM_SPEED__TRAITS_HPP_
