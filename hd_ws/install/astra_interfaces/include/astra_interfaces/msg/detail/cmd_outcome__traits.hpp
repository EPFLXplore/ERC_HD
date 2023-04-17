// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from astra_interfaces:msg/CmdOutcome.idl
// generated code does not contain a copyright notice

#ifndef ASTRA_INTERFACES__MSG__DETAIL__CMD_OUTCOME__TRAITS_HPP_
#define ASTRA_INTERFACES__MSG__DETAIL__CMD_OUTCOME__TRAITS_HPP_

#include "astra_interfaces/msg/detail/cmd_outcome__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<astra_interfaces::msg::CmdOutcome>()
{
  return "astra_interfaces::msg::CmdOutcome";
}

template<>
inline const char * name<astra_interfaces::msg::CmdOutcome>()
{
  return "astra_interfaces/msg/CmdOutcome";
}

template<>
struct has_fixed_size<astra_interfaces::msg::CmdOutcome>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<astra_interfaces::msg::CmdOutcome>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<astra_interfaces::msg::CmdOutcome>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ASTRA_INTERFACES__MSG__DETAIL__CMD_OUTCOME__TRAITS_HPP_
