// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from astra_interfaces:msg/Task.idl
// generated code does not contain a copyright notice

#ifndef ASTRA_INTERFACES__MSG__DETAIL__TASK__STRUCT_HPP_
#define ASTRA_INTERFACES__MSG__DETAIL__TASK__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__astra_interfaces__msg__Task __attribute__((deprecated))
#else
# define DEPRECATED__astra_interfaces__msg__Task __declspec(deprecated)
#endif

namespace astra_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Task_
{
  using Type = Task_<ContainerAllocator>;

  explicit Task_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : pose(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->description = "";
      this->id = 0;
    }
  }

  explicit Task_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : description(_alloc),
    pose(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->description = "";
      this->id = 0;
    }
  }

  // field types and members
  using _description_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _description_type description;
  using _id_type =
    int8_t;
  _id_type id;
  using _pose_type =
    geometry_msgs::msg::Pose_<ContainerAllocator>;
  _pose_type pose;

  // setters for named parameter idiom
  Type & set__description(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->description = _arg;
    return *this;
  }
  Type & set__id(
    const int8_t & _arg)
  {
    this->id = _arg;
    return *this;
  }
  Type & set__pose(
    const geometry_msgs::msg::Pose_<ContainerAllocator> & _arg)
  {
    this->pose = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    astra_interfaces::msg::Task_<ContainerAllocator> *;
  using ConstRawPtr =
    const astra_interfaces::msg::Task_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<astra_interfaces::msg::Task_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<astra_interfaces::msg::Task_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      astra_interfaces::msg::Task_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<astra_interfaces::msg::Task_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      astra_interfaces::msg::Task_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<astra_interfaces::msg::Task_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<astra_interfaces::msg::Task_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<astra_interfaces::msg::Task_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__astra_interfaces__msg__Task
    std::shared_ptr<astra_interfaces::msg::Task_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__astra_interfaces__msg__Task
    std::shared_ptr<astra_interfaces::msg::Task_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Task_ & other) const
  {
    if (this->description != other.description) {
      return false;
    }
    if (this->id != other.id) {
      return false;
    }
    if (this->pose != other.pose) {
      return false;
    }
    return true;
  }
  bool operator!=(const Task_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Task_

// alias to use template instance with default allocator
using Task =
  astra_interfaces::msg::Task_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace astra_interfaces

#endif  // ASTRA_INTERFACES__MSG__DETAIL__TASK__STRUCT_HPP_
