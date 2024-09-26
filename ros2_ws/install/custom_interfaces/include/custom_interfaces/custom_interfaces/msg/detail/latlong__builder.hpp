// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_interfaces:msg/Latlong.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACES__MSG__DETAIL__LATLONG__BUILDER_HPP_
#define CUSTOM_INTERFACES__MSG__DETAIL__LATLONG__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_interfaces/msg/detail/latlong__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_interfaces
{

namespace msg
{

namespace builder
{

class Init_Latlong_longitude
{
public:
  explicit Init_Latlong_longitude(::custom_interfaces::msg::Latlong & msg)
  : msg_(msg)
  {}
  ::custom_interfaces::msg::Latlong longitude(::custom_interfaces::msg::Latlong::_longitude_type arg)
  {
    msg_.longitude = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interfaces::msg::Latlong msg_;
};

class Init_Latlong_latitude
{
public:
  Init_Latlong_latitude()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Latlong_longitude latitude(::custom_interfaces::msg::Latlong::_latitude_type arg)
  {
    msg_.latitude = std::move(arg);
    return Init_Latlong_longitude(msg_);
  }

private:
  ::custom_interfaces::msg::Latlong msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interfaces::msg::Latlong>()
{
  return custom_interfaces::msg::builder::Init_Latlong_latitude();
}

}  // namespace custom_interfaces

#endif  // CUSTOM_INTERFACES__MSG__DETAIL__LATLONG__BUILDER_HPP_
