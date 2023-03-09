// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from astra_interfaces:msg/Object.idl
// generated code does not contain a copyright notice

#ifndef ASTRA_INTERFACES__MSG__DETAIL__OBJECT__BUILDER_HPP_
#define ASTRA_INTERFACES__MSG__DETAIL__OBJECT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "astra_interfaces/msg/detail/object__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace astra_interfaces
{

namespace msg
{

namespace builder
{

class Init_Object_dims
{
public:
  explicit Init_Object_dims(::astra_interfaces::msg::Object & msg)
  : msg_(msg)
  {}
  ::astra_interfaces::msg::Object dims(::astra_interfaces::msg::Object::_dims_type arg)
  {
    msg_.dims = std::move(arg);
    return std::move(msg_);
  }

private:
  ::astra_interfaces::msg::Object msg_;
};

class Init_Object_pose
{
public:
  explicit Init_Object_pose(::astra_interfaces::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_dims pose(::astra_interfaces::msg::Object::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return Init_Object_dims(msg_);
  }

private:
  ::astra_interfaces::msg::Object msg_;
};

class Init_Object_name
{
public:
  explicit Init_Object_name(::astra_interfaces::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_pose name(::astra_interfaces::msg::Object::_name_type arg)
  {
    msg_.name = std::move(arg);
    return Init_Object_pose(msg_);
  }

private:
  ::astra_interfaces::msg::Object msg_;
};

class Init_Object_type
{
public:
  Init_Object_type()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Object_name type(::astra_interfaces::msg::Object::_type_type arg)
  {
    msg_.type = std::move(arg);
    return Init_Object_name(msg_);
  }

private:
  ::astra_interfaces::msg::Object msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::astra_interfaces::msg::Object>()
{
  return astra_interfaces::msg::builder::Init_Object_type();
}

}  // namespace astra_interfaces

#endif  // ASTRA_INTERFACES__MSG__DETAIL__OBJECT__BUILDER_HPP_
