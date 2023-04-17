// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from astra_interfaces:msg/CmdOutcome.idl
// generated code does not contain a copyright notice

#ifndef ASTRA_INTERFACES__MSG__DETAIL__CMD_OUTCOME__BUILDER_HPP_
#define ASTRA_INTERFACES__MSG__DETAIL__CMD_OUTCOME__BUILDER_HPP_

#include "astra_interfaces/msg/detail/cmd_outcome__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace astra_interfaces
{

namespace msg
{

namespace builder
{

class Init_CmdOutcome_code
{
public:
  explicit Init_CmdOutcome_code(::astra_interfaces::msg::CmdOutcome & msg)
  : msg_(msg)
  {}
  ::astra_interfaces::msg::CmdOutcome code(::astra_interfaces::msg::CmdOutcome::_code_type arg)
  {
    msg_.code = std::move(arg);
    return std::move(msg_);
  }

private:
  ::astra_interfaces::msg::CmdOutcome msg_;
};

class Init_CmdOutcome_id
{
public:
  Init_CmdOutcome_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CmdOutcome_code id(::astra_interfaces::msg::CmdOutcome::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_CmdOutcome_code(msg_);
  }

private:
  ::astra_interfaces::msg::CmdOutcome msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::astra_interfaces::msg::CmdOutcome>()
{
  return astra_interfaces::msg::builder::Init_CmdOutcome_id();
}

}  // namespace astra_interfaces

#endif  // ASTRA_INTERFACES__MSG__DETAIL__CMD_OUTCOME__BUILDER_HPP_
