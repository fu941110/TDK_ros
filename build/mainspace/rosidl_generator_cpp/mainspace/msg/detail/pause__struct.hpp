// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mainspace:msg/Pause.idl
// generated code does not contain a copyright notice

#ifndef MAINSPACE__MSG__DETAIL__PAUSE__STRUCT_HPP_
#define MAINSPACE__MSG__DETAIL__PAUSE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__mainspace__msg__Pause __attribute__((deprecated))
#else
# define DEPRECATED__mainspace__msg__Pause __declspec(deprecated)
#endif

namespace mainspace
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Pause_
{
  using Type = Pause_<ContainerAllocator>;

  explicit Pause_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->pause = false;
    }
  }

  explicit Pause_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->pause = false;
    }
  }

  // field types and members
  using _pause_type =
    bool;
  _pause_type pause;

  // setters for named parameter idiom
  Type & set__pause(
    const bool & _arg)
  {
    this->pause = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mainspace::msg::Pause_<ContainerAllocator> *;
  using ConstRawPtr =
    const mainspace::msg::Pause_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mainspace::msg::Pause_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mainspace::msg::Pause_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mainspace::msg::Pause_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mainspace::msg::Pause_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mainspace::msg::Pause_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mainspace::msg::Pause_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mainspace::msg::Pause_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mainspace::msg::Pause_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mainspace__msg__Pause
    std::shared_ptr<mainspace::msg::Pause_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mainspace__msg__Pause
    std::shared_ptr<mainspace::msg::Pause_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Pause_ & other) const
  {
    if (this->pause != other.pause) {
      return false;
    }
    return true;
  }
  bool operator!=(const Pause_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Pause_

// alias to use template instance with default allocator
using Pause =
  mainspace::msg::Pause_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace mainspace

#endif  // MAINSPACE__MSG__DETAIL__PAUSE__STRUCT_HPP_
