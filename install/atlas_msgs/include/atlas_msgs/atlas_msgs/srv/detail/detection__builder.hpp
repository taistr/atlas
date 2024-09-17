// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from atlas_msgs:srv/Detection.idl
// generated code does not contain a copyright notice

#ifndef ATLAS_MSGS__SRV__DETAIL__DETECTION__BUILDER_HPP_
#define ATLAS_MSGS__SRV__DETAIL__DETECTION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "atlas_msgs/srv/detail/detection__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace atlas_msgs
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::atlas_msgs::srv::Detection_Request>()
{
  return ::atlas_msgs::srv::Detection_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace atlas_msgs


namespace atlas_msgs
{

namespace srv
{

namespace builder
{

class Init_Detection_Response_distance
{
public:
  explicit Init_Detection_Response_distance(::atlas_msgs::srv::Detection_Response & msg)
  : msg_(msg)
  {}
  ::atlas_msgs::srv::Detection_Response distance(::atlas_msgs::srv::Detection_Response::_distance_type arg)
  {
    msg_.distance = std::move(arg);
    return std::move(msg_);
  }

private:
  ::atlas_msgs::srv::Detection_Response msg_;
};

class Init_Detection_Response_angle
{
public:
  explicit Init_Detection_Response_angle(::atlas_msgs::srv::Detection_Response & msg)
  : msg_(msg)
  {}
  Init_Detection_Response_distance angle(::atlas_msgs::srv::Detection_Response::_angle_type arg)
  {
    msg_.angle = std::move(arg);
    return Init_Detection_Response_distance(msg_);
  }

private:
  ::atlas_msgs::srv::Detection_Response msg_;
};

class Init_Detection_Response_detection
{
public:
  Init_Detection_Response_detection()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Detection_Response_angle detection(::atlas_msgs::srv::Detection_Response::_detection_type arg)
  {
    msg_.detection = std::move(arg);
    return Init_Detection_Response_angle(msg_);
  }

private:
  ::atlas_msgs::srv::Detection_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::atlas_msgs::srv::Detection_Response>()
{
  return atlas_msgs::srv::builder::Init_Detection_Response_detection();
}

}  // namespace atlas_msgs

#endif  // ATLAS_MSGS__SRV__DETAIL__DETECTION__BUILDER_HPP_
