// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mainspace:msg/Command.idl
// generated code does not contain a copyright notice

#ifndef MAINSPACE__MSG__DETAIL__COMMAND__BUILDER_HPP_
#define MAINSPACE__MSG__DETAIL__COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mainspace/msg/detail/command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mainspace
{

namespace msg
{

namespace builder
{

class Init_Command_info
{
public:
  Init_Command_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::mainspace::msg::Command info(::mainspace::msg::Command::_info_type arg)
  {
    msg_.info = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mainspace::msg::Command msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mainspace::msg::Command>()
{
  return mainspace::msg::builder::Init_Command_info();
}

}  // namespace mainspace

#endif  // MAINSPACE__MSG__DETAIL__COMMAND__BUILDER_HPP_
