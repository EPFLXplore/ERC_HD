// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from interfaces:msg/PanelObject.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__PANEL_OBJECT__TRAITS_HPP_
#define INTERFACES__MSG__DETAIL__PANEL_OBJECT__TRAITS_HPP_

#include "interfaces/msg/detail/panel_object__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<interfaces::msg::PanelObject>()
{
  return "interfaces::msg::PanelObject";
}

template<>
inline const char * name<interfaces::msg::PanelObject>()
{
  return "interfaces/msg/PanelObject";
}

template<>
struct has_fixed_size<interfaces::msg::PanelObject>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Pose>::value> {};

template<>
struct has_bounded_size<interfaces::msg::PanelObject>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Pose>::value> {};

template<>
struct is_message<interfaces::msg::PanelObject>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // INTERFACES__MSG__DETAIL__PANEL_OBJECT__TRAITS_HPP_
