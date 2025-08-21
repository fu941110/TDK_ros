// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mainspace:msg/EncoderSpeed.idl
// generated code does not contain a copyright notice

#ifndef MAINSPACE__MSG__DETAIL__ENCODER_SPEED__TRAITS_HPP_
#define MAINSPACE__MSG__DETAIL__ENCODER_SPEED__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mainspace/msg/detail/encoder_speed__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace mainspace
{

namespace msg
{

inline void to_flow_style_yaml(
  const EncoderSpeed & msg,
  std::ostream & out)
{
  out << "{";
  // member: fl
  {
    out << "fl: ";
    rosidl_generator_traits::value_to_yaml(msg.fl, out);
    out << ", ";
  }

  // member: fr
  {
    out << "fr: ";
    rosidl_generator_traits::value_to_yaml(msg.fr, out);
    out << ", ";
  }

  // member: rl
  {
    out << "rl: ";
    rosidl_generator_traits::value_to_yaml(msg.rl, out);
    out << ", ";
  }

  // member: rr
  {
    out << "rr: ";
    rosidl_generator_traits::value_to_yaml(msg.rr, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const EncoderSpeed & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: fl
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fl: ";
    rosidl_generator_traits::value_to_yaml(msg.fl, out);
    out << "\n";
  }

  // member: fr
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fr: ";
    rosidl_generator_traits::value_to_yaml(msg.fr, out);
    out << "\n";
  }

  // member: rl
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rl: ";
    rosidl_generator_traits::value_to_yaml(msg.rl, out);
    out << "\n";
  }

  // member: rr
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rr: ";
    rosidl_generator_traits::value_to_yaml(msg.rr, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const EncoderSpeed & msg, bool use_flow_style = false)
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
  const mainspace::msg::EncoderSpeed & msg,
  std::ostream & out, size_t indentation = 0)
{
  mainspace::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mainspace::msg::to_yaml() instead")]]
inline std::string to_yaml(const mainspace::msg::EncoderSpeed & msg)
{
  return mainspace::msg::to_yaml(msg);
}

template<>
inline const char * data_type<mainspace::msg::EncoderSpeed>()
{
  return "mainspace::msg::EncoderSpeed";
}

template<>
inline const char * name<mainspace::msg::EncoderSpeed>()
{
  return "mainspace/msg/EncoderSpeed";
}

template<>
struct has_fixed_size<mainspace::msg::EncoderSpeed>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<mainspace::msg::EncoderSpeed>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<mainspace::msg::EncoderSpeed>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MAINSPACE__MSG__DETAIL__ENCODER_SPEED__TRAITS_HPP_
