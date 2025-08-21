// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mainspace:msg/Coffee.idl
// generated code does not contain a copyright notice

#ifndef MAINSPACE__MSG__DETAIL__COFFEE__STRUCT_HPP_
#define MAINSPACE__MSG__DETAIL__COFFEE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__mainspace__msg__Coffee __attribute__((deprecated))
#else
# define DEPRECATED__mainspace__msg__Coffee __declspec(deprecated)
#endif

namespace mainspace
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Coffee_
{
  using Type = Coffee_<ContainerAllocator>;

  explicit Coffee_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->type = "";
      this->number = 0l;
    }
  }

  explicit Coffee_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : type(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->type = "";
      this->number = 0l;
    }
  }

  // field types and members
  using _type_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _type_type type;
  using _number_type =
    int32_t;
  _number_type number;

  // setters for named parameter idiom
  Type & set__type(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->type = _arg;
    return *this;
  }
  Type & set__number(
    const int32_t & _arg)
  {
    this->number = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mainspace::msg::Coffee_<ContainerAllocator> *;
  using ConstRawPtr =
    const mainspace::msg::Coffee_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mainspace::msg::Coffee_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mainspace::msg::Coffee_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mainspace::msg::Coffee_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mainspace::msg::Coffee_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mainspace::msg::Coffee_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mainspace::msg::Coffee_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mainspace::msg::Coffee_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mainspace::msg::Coffee_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mainspace__msg__Coffee
    std::shared_ptr<mainspace::msg::Coffee_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mainspace__msg__Coffee
    std::shared_ptr<mainspace::msg::Coffee_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Coffee_ & other) const
  {
    if (this->type != other.type) {
      return false;
    }
    if (this->number != other.number) {
      return false;
    }
    return true;
  }
  bool operator!=(const Coffee_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Coffee_

// alias to use template instance with default allocator
using Coffee =
  mainspace::msg::Coffee_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace mainspace

#endif  // MAINSPACE__MSG__DETAIL__COFFEE__STRUCT_HPP_
