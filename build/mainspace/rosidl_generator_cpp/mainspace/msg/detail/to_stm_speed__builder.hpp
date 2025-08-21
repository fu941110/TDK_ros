// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mainspace:msg/ToStmSpeed.idl
// generated code does not contain a copyright notice

#ifndef MAINSPACE__MSG__DETAIL__TO_STM_SPEED__BUILDER_HPP_
#define MAINSPACE__MSG__DETAIL__TO_STM_SPEED__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mainspace/msg/detail/to_stm_speed__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mainspace
{

namespace msg
{

namespace builder
{

class Init_ToStmSpeed_w
{
public:
  explicit Init_ToStmSpeed_w(::mainspace::msg::ToStmSpeed & msg)
  : msg_(msg)
  {}
  ::mainspace::msg::ToStmSpeed w(::mainspace::msg::ToStmSpeed::_w_type arg)
  {
    msg_.w = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mainspace::msg::ToStmSpeed msg_;
};

class Init_ToStmSpeed_vy
{
public:
  explicit Init_ToStmSpeed_vy(::mainspace::msg::ToStmSpeed & msg)
  : msg_(msg)
  {}
  Init_ToStmSpeed_w vy(::mainspace::msg::ToStmSpeed::_vy_type arg)
  {
    msg_.vy = std::move(arg);
    return Init_ToStmSpeed_w(msg_);
  }

private:
  ::mainspace::msg::ToStmSpeed msg_;
};

class Init_ToStmSpeed_vx
{
public:
  Init_ToStmSpeed_vx()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ToStmSpeed_vy vx(::mainspace::msg::ToStmSpeed::_vx_type arg)
  {
    msg_.vx = std::move(arg);
    return Init_ToStmSpeed_vy(msg_);
  }

private:
  ::mainspace::msg::ToStmSpeed msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mainspace::msg::ToStmSpeed>()
{
  return mainspace::msg::builder::Init_ToStmSpeed_vx();
}

}  // namespace mainspace

#endif  // MAINSPACE__MSG__DETAIL__TO_STM_SPEED__BUILDER_HPP_
