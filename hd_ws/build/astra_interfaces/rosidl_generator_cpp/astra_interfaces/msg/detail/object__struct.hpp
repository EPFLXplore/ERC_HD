// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from astra_interfaces:msg/Object.idl
// generated code does not contain a copyright notice

#ifndef ASTRA_INTERFACES__MSG__DETAIL__OBJECT__STRUCT_HPP_
#define ASTRA_INTERFACES__MSG__DETAIL__OBJECT__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__struct.hpp"
// Member 'dims'
#include "std_msgs/msg/detail/float32_multi_array__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__astra_interfaces__msg__Object __attribute__((deprecated))
#else
# define DEPRECATED__astra_interfaces__msg__Object __declspec(deprecated)
#endif

namespace astra_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Object_
{
  using Type = Object_<ContainerAllocator>;

  explicit Object_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : pose(_init),
    dims(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->type = "";
      this->name = "";
    }
  }

  explicit Object_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : type(_alloc),
    name(_alloc),
    pose(_alloc, _init),
    dims(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->type = "";
      this->name = "";
    }
  }

  // field types and members
  using _type_type =
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>;
  _type_type type;
  using _name_type =
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>;
  _name_type name;
  using _pose_type =
    geometry_msgs::msg::Pose_<ContainerAllocator>;
  _pose_type pose;
  using _dims_type =
    std_msgs::msg::Float32MultiArray_<ContainerAllocator>;
  _dims_type dims;

  // setters for named parameter idiom
  Type & set__type(
    const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other> & _arg)
  {
    this->type = _arg;
    return *this;
  }
  Type & set__name(
    const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other> & _arg)
  {
    this->name = _arg;
    return *this;
  }
  Type & set__pose(
    const geometry_msgs::msg::Pose_<ContainerAllocator> & _arg)
  {
    this->pose = _arg;
    return *this;
  }
  Type & set__dims(
    const std_msgs::msg::Float32MultiArray_<ContainerAllocator> & _arg)
  {
    this->dims = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    astra_interfaces::msg::Object_<ContainerAllocator> *;
  using ConstRawPtr =
    const astra_interfaces::msg::Object_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<astra_interfaces::msg::Object_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<astra_interfaces::msg::Object_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      astra_interfaces::msg::Object_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<astra_interfaces::msg::Object_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      astra_interfaces::msg::Object_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<astra_interfaces::msg::Object_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<astra_interfaces::msg::Object_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<astra_interfaces::msg::Object_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__astra_interfaces__msg__Object
    std::shared_ptr<astra_interfaces::msg::Object_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__astra_interfaces__msg__Object
    std::shared_ptr<astra_interfaces::msg::Object_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Object_ & other) const
  {
    if (this->type != other.type) {
      return false;
    }
    if (this->name != other.name) {
      return false;
    }
    if (this->pose != other.pose) {
      return false;
    }
    if (this->dims != other.dims) {
      return false;
    }
    return true;
  }
  bool operator!=(const Object_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Object_

// alias to use template instance with default allocator
using Object =
  astra_interfaces::msg::Object_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace astra_interfaces

#endif  // ASTRA_INTERFACES__MSG__DETAIL__OBJECT__STRUCT_HPP_
