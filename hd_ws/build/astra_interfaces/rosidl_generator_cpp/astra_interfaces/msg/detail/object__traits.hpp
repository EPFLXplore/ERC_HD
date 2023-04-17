// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from astra_interfaces:msg/Object.idl
// generated code does not contain a copyright notice

#ifndef ASTRA_INTERFACES__MSG__DETAIL__OBJECT__TRAITS_HPP_
#define ASTRA_INTERFACES__MSG__DETAIL__OBJECT__TRAITS_HPP_

#include "astra_interfaces/msg/detail/object__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__traits.hpp"
// Member 'dims'
#include "std_msgs/msg/detail/float32_multi_array__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<astra_interfaces::msg::Object>()
{
  return "astra_interfaces::msg::Object";
}

template<>
inline const char * name<astra_interfaces::msg::Object>()
{
  return "astra_interfaces/msg/Object";
}

template<>
struct has_fixed_size<astra_interfaces::msg::Object>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<astra_interfaces::msg::Object>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<astra_interfaces::msg::Object>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ASTRA_INTERFACES__MSG__DETAIL__OBJECT__TRAITS_HPP_
