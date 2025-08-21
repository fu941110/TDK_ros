// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mainspace:msg/Pause.idl
// generated code does not contain a copyright notice

#ifndef MAINSPACE__MSG__DETAIL__PAUSE__BUILDER_HPP_
#define MAINSPACE__MSG__DETAIL__PAUSE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mainspace/msg/detail/pause__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mainspace
{

namespace msg
{

namespace builder
{

class Init_Pause_pause
{
public:
  Init_Pause_pause()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::mainspace::msg::Pause pause(::mainspace::msg::Pause::_pause_type arg)
  {
    msg_.pause = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mainspace::msg::Pause msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mainspace::msg::Pause>()
{
  return mainspace::msg::builder::Init_Pause_pause();
}

}  // namespace mainspace

#endif  // MAINSPACE__MSG__DETAIL__PAUSE__BUILDER_HPP_
