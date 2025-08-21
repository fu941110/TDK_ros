// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mainspace:msg/EncoderSpeed.idl
// generated code does not contain a copyright notice

#ifndef MAINSPACE__MSG__DETAIL__ENCODER_SPEED__BUILDER_HPP_
#define MAINSPACE__MSG__DETAIL__ENCODER_SPEED__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mainspace/msg/detail/encoder_speed__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mainspace
{

namespace msg
{

namespace builder
{

class Init_EncoderSpeed_rr
{
public:
  explicit Init_EncoderSpeed_rr(::mainspace::msg::EncoderSpeed & msg)
  : msg_(msg)
  {}
  ::mainspace::msg::EncoderSpeed rr(::mainspace::msg::EncoderSpeed::_rr_type arg)
  {
    msg_.rr = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mainspace::msg::EncoderSpeed msg_;
};

class Init_EncoderSpeed_rl
{
public:
  explicit Init_EncoderSpeed_rl(::mainspace::msg::EncoderSpeed & msg)
  : msg_(msg)
  {}
  Init_EncoderSpeed_rr rl(::mainspace::msg::EncoderSpeed::_rl_type arg)
  {
    msg_.rl = std::move(arg);
    return Init_EncoderSpeed_rr(msg_);
  }

private:
  ::mainspace::msg::EncoderSpeed msg_;
};

class Init_EncoderSpeed_fr
{
public:
  explicit Init_EncoderSpeed_fr(::mainspace::msg::EncoderSpeed & msg)
  : msg_(msg)
  {}
  Init_EncoderSpeed_rl fr(::mainspace::msg::EncoderSpeed::_fr_type arg)
  {
    msg_.fr = std::move(arg);
    return Init_EncoderSpeed_rl(msg_);
  }

private:
  ::mainspace::msg::EncoderSpeed msg_;
};

class Init_EncoderSpeed_fl
{
public:
  Init_EncoderSpeed_fl()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_EncoderSpeed_fr fl(::mainspace::msg::EncoderSpeed::_fl_type arg)
  {
    msg_.fl = std::move(arg);
    return Init_EncoderSpeed_fr(msg_);
  }

private:
  ::mainspace::msg::EncoderSpeed msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mainspace::msg::EncoderSpeed>()
{
  return mainspace::msg::builder::Init_EncoderSpeed_fl();
}

}  // namespace mainspace

#endif  // MAINSPACE__MSG__DETAIL__ENCODER_SPEED__BUILDER_HPP_
