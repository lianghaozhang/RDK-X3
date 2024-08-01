// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ackermann_msgs:msg/AckermannDrive.idl
// generated code does not contain a copyright notice

#ifndef ACKERMANN_MSGS__MSG__DETAIL__ACKERMANN_DRIVE__TRAITS_HPP_
#define ACKERMANN_MSGS__MSG__DETAIL__ACKERMANN_DRIVE__TRAITS_HPP_

#include "ackermann_msgs/msg/detail/ackermann_drive__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<ackermann_msgs::msg::AckermannDrive>()
{
  return "ackermann_msgs::msg::AckermannDrive";
}

template<>
inline const char * name<ackermann_msgs::msg::AckermannDrive>()
{
  return "ackermann_msgs/msg/AckermannDrive";
}

template<>
struct has_fixed_size<ackermann_msgs::msg::AckermannDrive>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<ackermann_msgs::msg::AckermannDrive>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<ackermann_msgs::msg::AckermannDrive>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ACKERMANN_MSGS__MSG__DETAIL__ACKERMANN_DRIVE__TRAITS_HPP_
