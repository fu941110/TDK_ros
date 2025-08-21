// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mainspace:msg/CsvFile.idl
// generated code does not contain a copyright notice

#ifndef MAINSPACE__MSG__DETAIL__CSV_FILE__BUILDER_HPP_
#define MAINSPACE__MSG__DETAIL__CSV_FILE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mainspace/msg/detail/csv_file__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mainspace
{

namespace msg
{

namespace builder
{

class Init_CsvFile_file
{
public:
  Init_CsvFile_file()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::mainspace::msg::CsvFile file(::mainspace::msg::CsvFile::_file_type arg)
  {
    msg_.file = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mainspace::msg::CsvFile msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mainspace::msg::CsvFile>()
{
  return mainspace::msg::builder::Init_CsvFile_file();
}

}  // namespace mainspace

#endif  // MAINSPACE__MSG__DETAIL__CSV_FILE__BUILDER_HPP_
