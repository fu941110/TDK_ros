// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mainspace:msg/CsvFile.idl
// generated code does not contain a copyright notice

#ifndef MAINSPACE__MSG__DETAIL__CSV_FILE__STRUCT_HPP_
#define MAINSPACE__MSG__DETAIL__CSV_FILE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__mainspace__msg__CsvFile __attribute__((deprecated))
#else
# define DEPRECATED__mainspace__msg__CsvFile __declspec(deprecated)
#endif

namespace mainspace
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct CsvFile_
{
  using Type = CsvFile_<ContainerAllocator>;

  explicit CsvFile_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->file = "";
    }
  }

  explicit CsvFile_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : file(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->file = "";
    }
  }

  // field types and members
  using _file_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _file_type file;

  // setters for named parameter idiom
  Type & set__file(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->file = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mainspace::msg::CsvFile_<ContainerAllocator> *;
  using ConstRawPtr =
    const mainspace::msg::CsvFile_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mainspace::msg::CsvFile_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mainspace::msg::CsvFile_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mainspace::msg::CsvFile_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mainspace::msg::CsvFile_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mainspace::msg::CsvFile_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mainspace::msg::CsvFile_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mainspace::msg::CsvFile_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mainspace::msg::CsvFile_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mainspace__msg__CsvFile
    std::shared_ptr<mainspace::msg::CsvFile_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mainspace__msg__CsvFile
    std::shared_ptr<mainspace::msg::CsvFile_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CsvFile_ & other) const
  {
    if (this->file != other.file) {
      return false;
    }
    return true;
  }
  bool operator!=(const CsvFile_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CsvFile_

// alias to use template instance with default allocator
using CsvFile =
  mainspace::msg::CsvFile_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace mainspace

#endif  // MAINSPACE__MSG__DETAIL__CSV_FILE__STRUCT_HPP_
