// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from astra_interfaces:msg/CmdOutcome.idl
// generated code does not contain a copyright notice

#ifndef ASTRA_INTERFACES__MSG__DETAIL__CMD_OUTCOME__TRAITS_HPP_
#define ASTRA_INTERFACES__MSG__DETAIL__CMD_OUTCOME__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "astra_interfaces/msg/detail/cmd_outcome__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace astra_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const CmdOutcome & msg,
  std::ostream & out)
{
  out << "{";
  // member: id
  {
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << ", ";
  }

  // member: code
  {
    out << "code: ";
    rosidl_generator_traits::value_to_yaml(msg.code, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CmdOutcome & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << "\n";
  }

  // member: code
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "code: ";
    rosidl_generator_traits::value_to_yaml(msg.code, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CmdOutcome & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace astra_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use astra_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const astra_interfaces::msg::CmdOutcome & msg,
  std::ostream & out, size_t indentation = 0)
{
  astra_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use astra_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const astra_interfaces::msg::CmdOutcome & msg)
{
  return astra_interfaces::msg::to_yaml(msg);
}

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
