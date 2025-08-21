// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mainspace:msg/Desk.idl
// generated code does not contain a copyright notice

#ifndef MAINSPACE__MSG__DETAIL__DESK__BUILDER_HPP_
#define MAINSPACE__MSG__DETAIL__DESK__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mainspace/msg/detail/desk__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mainspace
{

namespace msg
{

namespace builder
{

class Init_Desk_y
{
public:
  explicit Init_Desk_y(::mainspace::msg::Desk & msg)
  : msg_(msg)
  {}
  ::mainspace::msg::Desk y(::mainspace::msg::Desk::_y_type arg)
  {
    msg_.y = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mainspace::msg::Desk msg_;
};

class Init_Desk_x
{
public:
  Init_Desk_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Desk_y x(::mainspace::msg::Desk::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_Desk_y(msg_);
  }

private:
  ::mainspace::msg::Desk msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mainspace::msg::Desk>()
{
  return mainspace::msg::builder::Init_Desk_x();
}

}  // namespace mainspace

#endif  // MAINSPACE__MSG__DETAIL__DESK__BUILDER_HPP_
