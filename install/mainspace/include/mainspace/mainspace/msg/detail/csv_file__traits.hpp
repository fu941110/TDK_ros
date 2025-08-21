// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mainspace:msg/CsvFile.idl
// generated code does not contain a copyright notice

#ifndef MAINSPACE__MSG__DETAIL__CSV_FILE__TRAITS_HPP_
#define MAINSPACE__MSG__DETAIL__CSV_FILE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mainspace/msg/detail/csv_file__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace mainspace
{

namespace msg
{

inline void to_flow_style_yaml(
  const CsvFile & msg,
  std::ostream & out)
{
  out << "{";
  // member: file
  {
    out << "file: ";
    rosidl_generator_traits::value_to_yaml(msg.file, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CsvFile & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: file
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "file: ";
    rosidl_generator_traits::value_to_yaml(msg.file, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CsvFile & msg, bool use_flow_style = false)
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
  const mainspace::msg::CsvFile & msg,
  std::ostream & out, size_t indentation = 0)
{
  mainspace::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mainspace::msg::to_yaml() instead")]]
inline std::string to_yaml(const mainspace::msg::CsvFile & msg)
{
  return mainspace::msg::to_yaml(msg);
}

template<>
inline const char * data_type<mainspace::msg::CsvFile>()
{
  return "mainspace::msg::CsvFile";
}

template<>
inline const char * name<mainspace::msg::CsvFile>()
{
  return "mainspace/msg/CsvFile";
}

template<>
struct has_fixed_size<mainspace::msg::CsvFile>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<mainspace::msg::CsvFile>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<mainspace::msg::CsvFile>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MAINSPACE__MSG__DETAIL__CSV_FILE__TRAITS_HPP_
