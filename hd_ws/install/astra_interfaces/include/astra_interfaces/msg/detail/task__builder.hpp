// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from astra_interfaces:msg/Task.idl
// generated code does not contain a copyright notice

#ifndef ASTRA_INTERFACES__MSG__DETAIL__TASK__BUILDER_HPP_
#define ASTRA_INTERFACES__MSG__DETAIL__TASK__BUILDER_HPP_

#include "astra_interfaces/msg/detail/task__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace astra_interfaces
{

namespace msg
{

namespace builder
{

class Init_Task_pose
{
public:
  explicit Init_Task_pose(::astra_interfaces::msg::Task & msg)
  : msg_(msg)
  {}
  ::astra_interfaces::msg::Task pose(::astra_interfaces::msg::Task::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return std::move(msg_);
  }

private:
  ::astra_interfaces::msg::Task msg_;
};

class Init_Task_id
{
public:
  explicit Init_Task_id(::astra_interfaces::msg::Task & msg)
  : msg_(msg)
  {}
  Init_Task_pose id(::astra_interfaces::msg::Task::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_Task_pose(msg_);
  }

private:
  ::astra_interfaces::msg::Task msg_;
};

class Init_Task_description
{
public:
  Init_Task_description()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Task_id description(::astra_interfaces::msg::Task::_description_type arg)
  {
    msg_.description = std::move(arg);
    return Init_Task_id(msg_);
  }

private:
  ::astra_interfaces::msg::Task msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::astra_interfaces::msg::Task>()
{
  return astra_interfaces::msg::builder::Init_Task_description();
}

}  // namespace astra_interfaces

#endif  // ASTRA_INTERFACES__MSG__DETAIL__TASK__BUILDER_HPP_
