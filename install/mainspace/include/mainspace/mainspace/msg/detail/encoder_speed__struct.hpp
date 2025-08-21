// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mainspace:msg/EncoderSpeed.idl
// generated code does not contain a copyright notice

#ifndef MAINSPACE__MSG__DETAIL__ENCODER_SPEED__STRUCT_HPP_
#define MAINSPACE__MSG__DETAIL__ENCODER_SPEED__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__mainspace__msg__EncoderSpeed __attribute__((deprecated))
#else
# define DEPRECATED__mainspace__msg__EncoderSpeed __declspec(deprecated)
#endif

namespace mainspace
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct EncoderSpeed_
{
  using Type = EncoderSpeed_<ContainerAllocator>;

  explicit EncoderSpeed_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->fl = 0l;
      this->fr = 0l;
      this->rl = 0l;
      this->rr = 0l;
    }
  }

  explicit EncoderSpeed_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->fl = 0l;
      this->fr = 0l;
      this->rl = 0l;
      this->rr = 0l;
    }
  }

  // field types and members
  using _fl_type =
    int32_t;
  _fl_type fl;
  using _fr_type =
    int32_t;
  _fr_type fr;
  using _rl_type =
    int32_t;
  _rl_type rl;
  using _rr_type =
    int32_t;
  _rr_type rr;

  // setters for named parameter idiom
  Type & set__fl(
    const int32_t & _arg)
  {
    this->fl = _arg;
    return *this;
  }
  Type & set__fr(
    const int32_t & _arg)
  {
    this->fr = _arg;
    return *this;
  }
  Type & set__rl(
    const int32_t & _arg)
  {
    this->rl = _arg;
    return *this;
  }
  Type & set__rr(
    const int32_t & _arg)
  {
    this->rr = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mainspace::msg::EncoderSpeed_<ContainerAllocator> *;
  using ConstRawPtr =
    const mainspace::msg::EncoderSpeed_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mainspace::msg::EncoderSpeed_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mainspace::msg::EncoderSpeed_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mainspace::msg::EncoderSpeed_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mainspace::msg::EncoderSpeed_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mainspace::msg::EncoderSpeed_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mainspace::msg::EncoderSpeed_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mainspace::msg::EncoderSpeed_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mainspace::msg::EncoderSpeed_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mainspace__msg__EncoderSpeed
    std::shared_ptr<mainspace::msg::EncoderSpeed_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mainspace__msg__EncoderSpeed
    std::shared_ptr<mainspace::msg::EncoderSpeed_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const EncoderSpeed_ & other) const
  {
    if (this->fl != other.fl) {
      return false;
    }
    if (this->fr != other.fr) {
      return false;
    }
    if (this->rl != other.rl) {
      return false;
    }
    if (this->rr != other.rr) {
      return false;
    }
    return true;
  }
  bool operator!=(const EncoderSpeed_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct EncoderSpeed_

// alias to use template instance with default allocator
using EncoderSpeed =
  mainspace::msg::EncoderSpeed_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace mainspace

#endif  // MAINSPACE__MSG__DETAIL__ENCODER_SPEED__STRUCT_HPP_
