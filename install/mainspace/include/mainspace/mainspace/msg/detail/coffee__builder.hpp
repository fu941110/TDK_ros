// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mainspace:msg/Coffee.idl
// generated code does not contain a copyright notice

#ifndef MAINSPACE__MSG__DETAIL__COFFEE__BUILDER_HPP_
#define MAINSPACE__MSG__DETAIL__COFFEE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mainspace/msg/detail/coffee__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mainspace
{

namespace msg
{

namespace builder
{

class Init_Coffee_number
{
public:
  explicit Init_Coffee_number(::mainspace::msg::Coffee & msg)
  : msg_(msg)
  {}
  ::mainspace::msg::Coffee number(::mainspace::msg::Coffee::_number_type arg)
  {
    msg_.number = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mainspace::msg::Coffee msg_;
};

class Init_Coffee_type
{
public:
  Init_Coffee_type()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Coffee_number type(::mainspace::msg::Coffee::_type_type arg)
  {
    msg_.type = std::move(arg);
    return Init_Coffee_number(msg_);
  }

private:
  ::mainspace::msg::Coffee msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mainspace::msg::Coffee>()
{
  return mainspace::msg::builder::Init_Coffee_type();
}

}  // namespace mainspace

#endif  // MAINSPACE__MSG__DETAIL__COFFEE__BUILDER_HPP_
