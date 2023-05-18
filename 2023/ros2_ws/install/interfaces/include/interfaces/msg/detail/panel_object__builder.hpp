// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from interfaces:msg/PanelObject.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__PANEL_OBJECT__BUILDER_HPP_
#define INTERFACES__MSG__DETAIL__PANEL_OBJECT__BUILDER_HPP_

#include "interfaces/msg/detail/panel_object__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace interfaces
{

namespace msg
{

namespace builder
{

class Init_PanelObject_pose
{
public:
  explicit Init_PanelObject_pose(::interfaces::msg::PanelObject & msg)
  : msg_(msg)
  {}
  ::interfaces::msg::PanelObject pose(::interfaces::msg::PanelObject::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interfaces::msg::PanelObject msg_;
};

class Init_PanelObject_id
{
public:
  Init_PanelObject_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PanelObject_pose id(::interfaces::msg::PanelObject::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_PanelObject_pose(msg_);
  }

private:
  ::interfaces::msg::PanelObject msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::interfaces::msg::PanelObject>()
{
  return interfaces::msg::builder::Init_PanelObject_id();
}

}  // namespace interfaces

#endif  // INTERFACES__MSG__DETAIL__PANEL_OBJECT__BUILDER_HPP_
