// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from atlas_msgs:srv/MotionRequest.idl
// generated code does not contain a copyright notice

#ifndef ATLAS_MSGS__SRV__DETAIL__MOTION_REQUEST__TRAITS_HPP_
#define ATLAS_MSGS__SRV__DETAIL__MOTION_REQUEST__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "atlas_msgs/srv/detail/motion_request__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace atlas_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const MotionRequest_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: angle
  {
    out << "angle: ";
    rosidl_generator_traits::value_to_yaml(msg.angle, out);
    out << ", ";
  }

  // member: distance
  {
    out << "distance: ";
    rosidl_generator_traits::value_to_yaml(msg.distance, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MotionRequest_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: angle
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "angle: ";
    rosidl_generator_traits::value_to_yaml(msg.angle, out);
    out << "\n";
  }

  // member: distance
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "distance: ";
    rosidl_generator_traits::value_to_yaml(msg.distance, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MotionRequest_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace atlas_msgs

namespace rosidl_generator_traits
{

[[deprecated("use atlas_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const atlas_msgs::srv::MotionRequest_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  atlas_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use atlas_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const atlas_msgs::srv::MotionRequest_Request & msg)
{
  return atlas_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<atlas_msgs::srv::MotionRequest_Request>()
{
  return "atlas_msgs::srv::MotionRequest_Request";
}

template<>
inline const char * name<atlas_msgs::srv::MotionRequest_Request>()
{
  return "atlas_msgs/srv/MotionRequest_Request";
}

template<>
struct has_fixed_size<atlas_msgs::srv::MotionRequest_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<atlas_msgs::srv::MotionRequest_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<atlas_msgs::srv::MotionRequest_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace atlas_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const MotionRequest_Response & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MotionRequest_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MotionRequest_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace atlas_msgs

namespace rosidl_generator_traits
{

[[deprecated("use atlas_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const atlas_msgs::srv::MotionRequest_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  atlas_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use atlas_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const atlas_msgs::srv::MotionRequest_Response & msg)
{
  return atlas_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<atlas_msgs::srv::MotionRequest_Response>()
{
  return "atlas_msgs::srv::MotionRequest_Response";
}

template<>
inline const char * name<atlas_msgs::srv::MotionRequest_Response>()
{
  return "atlas_msgs/srv/MotionRequest_Response";
}

template<>
struct has_fixed_size<atlas_msgs::srv::MotionRequest_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<atlas_msgs::srv::MotionRequest_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<atlas_msgs::srv::MotionRequest_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<atlas_msgs::srv::MotionRequest>()
{
  return "atlas_msgs::srv::MotionRequest";
}

template<>
inline const char * name<atlas_msgs::srv::MotionRequest>()
{
  return "atlas_msgs/srv/MotionRequest";
}

template<>
struct has_fixed_size<atlas_msgs::srv::MotionRequest>
  : std::integral_constant<
    bool,
    has_fixed_size<atlas_msgs::srv::MotionRequest_Request>::value &&
    has_fixed_size<atlas_msgs::srv::MotionRequest_Response>::value
  >
{
};

template<>
struct has_bounded_size<atlas_msgs::srv::MotionRequest>
  : std::integral_constant<
    bool,
    has_bounded_size<atlas_msgs::srv::MotionRequest_Request>::value &&
    has_bounded_size<atlas_msgs::srv::MotionRequest_Response>::value
  >
{
};

template<>
struct is_service<atlas_msgs::srv::MotionRequest>
  : std::true_type
{
};

template<>
struct is_service_request<atlas_msgs::srv::MotionRequest_Request>
  : std::true_type
{
};

template<>
struct is_service_response<atlas_msgs::srv::MotionRequest_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // ATLAS_MSGS__SRV__DETAIL__MOTION_REQUEST__TRAITS_HPP_
