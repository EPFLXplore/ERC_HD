// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from astra_interfaces:msg/CmdOutcome.idl
// generated code does not contain a copyright notice

#ifndef ASTRA_INTERFACES__MSG__DETAIL__CMD_OUTCOME__STRUCT_HPP_
#define ASTRA_INTERFACES__MSG__DETAIL__CMD_OUTCOME__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__astra_interfaces__msg__CmdOutcome __attribute__((deprecated))
#else
# define DEPRECATED__astra_interfaces__msg__CmdOutcome __declspec(deprecated)
#endif

namespace astra_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct CmdOutcome_
{
  using Type = CmdOutcome_<ContainerAllocator>;

  explicit CmdOutcome_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0;
      this->code = 0;
    }
  }

  explicit CmdOutcome_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0;
      this->code = 0;
    }
  }

  // field types and members
  using _id_type =
    uint16_t;
  _id_type id;
  using _code_type =
    uint8_t;
  _code_type code;

  // setters for named parameter idiom
  Type & set__id(
    const uint16_t & _arg)
  {
    this->id = _arg;
    return *this;
  }
  Type & set__code(
    const uint8_t & _arg)
  {
    this->code = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    astra_interfaces::msg::CmdOutcome_<ContainerAllocator> *;
  using ConstRawPtr =
    const astra_interfaces::msg::CmdOutcome_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<astra_interfaces::msg::CmdOutcome_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<astra_interfaces::msg::CmdOutcome_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      astra_interfaces::msg::CmdOutcome_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<astra_interfaces::msg::CmdOutcome_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      astra_interfaces::msg::CmdOutcome_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<astra_interfaces::msg::CmdOutcome_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<astra_interfaces::msg::CmdOutcome_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<astra_interfaces::msg::CmdOutcome_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__astra_interfaces__msg__CmdOutcome
    std::shared_ptr<astra_interfaces::msg::CmdOutcome_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__astra_interfaces__msg__CmdOutcome
    std::shared_ptr<astra_interfaces::msg::CmdOutcome_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CmdOutcome_ & other) const
  {
    if (this->id != other.id) {
      return false;
    }
    if (this->code != other.code) {
      return false;
    }
    return true;
  }
  bool operator!=(const CmdOutcome_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CmdOutcome_

// alias to use template instance with default allocator
using CmdOutcome =
  astra_interfaces::msg::CmdOutcome_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace astra_interfaces

#endif  // ASTRA_INTERFACES__MSG__DETAIL__CMD_OUTCOME__STRUCT_HPP_
