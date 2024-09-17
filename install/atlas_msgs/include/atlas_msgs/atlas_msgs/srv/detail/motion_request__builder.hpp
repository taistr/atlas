// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from atlas_msgs:srv/MotionRequest.idl
// generated code does not contain a copyright notice

#ifndef ATLAS_MSGS__SRV__DETAIL__MOTION_REQUEST__BUILDER_HPP_
#define ATLAS_MSGS__SRV__DETAIL__MOTION_REQUEST__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "atlas_msgs/srv/detail/motion_request__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace atlas_msgs
{

namespace srv
{

namespace builder
{

class Init_MotionRequest_Request_distance
{
public:
  explicit Init_MotionRequest_Request_distance(::atlas_msgs::srv::MotionRequest_Request & msg)
  : msg_(msg)
  {}
  ::atlas_msgs::srv::MotionRequest_Request distance(::atlas_msgs::srv::MotionRequest_Request::_distance_type arg)
  {
    msg_.distance = std::move(arg);
    return std::move(msg_);
  }

private:
  ::atlas_msgs::srv::MotionRequest_Request msg_;
};

class Init_MotionRequest_Request_angle
{
public:
  Init_MotionRequest_Request_angle()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MotionRequest_Request_distance angle(::atlas_msgs::srv::MotionRequest_Request::_angle_type arg)
  {
    msg_.angle = std::move(arg);
    return Init_MotionRequest_Request_distance(msg_);
  }

private:
  ::atlas_msgs::srv::MotionRequest_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::atlas_msgs::srv::MotionRequest_Request>()
{
  return atlas_msgs::srv::builder::Init_MotionRequest_Request_angle();
}

}  // namespace atlas_msgs


namespace atlas_msgs
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::atlas_msgs::srv::MotionRequest_Response>()
{
  return ::atlas_msgs::srv::MotionRequest_Response(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace atlas_msgs

#endif  // ATLAS_MSGS__SRV__DETAIL__MOTION_REQUEST__BUILDER_HPP_
