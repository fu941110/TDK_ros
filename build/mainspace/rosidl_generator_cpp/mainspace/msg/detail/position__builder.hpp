// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mainspace:msg/Position.idl
// generated code does not contain a copyright notice

#ifndef MAINSPACE__MSG__DETAIL__POSITION__BUILDER_HPP_
#define MAINSPACE__MSG__DETAIL__POSITION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mainspace/msg/detail/position__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mainspace
{

namespace msg
{

namespace builder
{

class Init_Position_theta
{
public:
  explicit Init_Position_theta(::mainspace::msg::Position & msg)
  : msg_(msg)
  {}
  ::mainspace::msg::Position theta(::mainspace::msg::Position::_theta_type arg)
  {
    msg_.theta = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mainspace::msg::Position msg_;
};

class Init_Position_y
{
public:
  explicit Init_Position_y(::mainspace::msg::Position & msg)
  : msg_(msg)
  {}
  Init_Position_theta y(::mainspace::msg::Position::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_Position_theta(msg_);
  }

private:
  ::mainspace::msg::Position msg_;
};

class Init_Position_x
{
public:
  Init_Position_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Position_y x(::mainspace::msg::Position::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_Position_y(msg_);
  }

private:
  ::mainspace::msg::Position msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mainspace::msg::Position>()
{
  return mainspace::msg::builder::Init_Position_x();
}

}  // namespace mainspace

#endif  // MAINSPACE__MSG__DETAIL__POSITION__BUILDER_HPP_
