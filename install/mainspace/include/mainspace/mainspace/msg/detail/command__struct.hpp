// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mainspace:msg/Command.idl
// generated code does not contain a copyright notice

#ifndef MAINSPACE__MSG__DETAIL__COMMAND__STRUCT_HPP_
#define MAINSPACE__MSG__DETAIL__COMMAND__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__mainspace__msg__Command __attribute__((deprecated))
#else
# define DEPRECATED__mainspace__msg__Command __declspec(deprecated)
#endif

namespace mainspace
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Command_
{
  using Type = Command_<ContainerAllocator>;

  explicit Command_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->info = "";
    }
  }

  explicit Command_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->info = "";
    }
  }

  // field types and members
  using _info_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _info_type info;

  // setters for named parameter idiom
  Type & set__info(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->info = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mainspace::msg::Command_<ContainerAllocator> *;
  using ConstRawPtr =
    const mainspace::msg::Command_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mainspace::msg::Command_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mainspace::msg::Command_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mainspace::msg::Command_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mainspace::msg::Command_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mainspace::msg::Command_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mainspace::msg::Command_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mainspace::msg::Command_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mainspace::msg::Command_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mainspace__msg__Command
    std::shared_ptr<mainspace::msg::Command_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mainspace__msg__Command
    std::shared_ptr<mainspace::msg::Command_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Command_ & other) const
  {
    if (this->info != other.info) {
      return false;
    }
    return true;
  }
  bool operator!=(const Command_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Command_

// alias to use template instance with default allocator
using Command =
  mainspace::msg::Command_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace mainspace

#endif  // MAINSPACE__MSG__DETAIL__COMMAND__STRUCT_HPP_
