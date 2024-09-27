// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from atlas_msgs:srv/MotionRequest.idl
// generated code does not contain a copyright notice

#ifndef ATLAS_MSGS__SRV__DETAIL__MOTION_REQUEST__STRUCT_HPP_
#define ATLAS_MSGS__SRV__DETAIL__MOTION_REQUEST__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__atlas_msgs__srv__MotionRequest_Request __attribute__((deprecated))
#else
# define DEPRECATED__atlas_msgs__srv__MotionRequest_Request __declspec(deprecated)
#endif

namespace atlas_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct MotionRequest_Request_
{
  using Type = MotionRequest_Request_<ContainerAllocator>;

  explicit MotionRequest_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->angle = 0.0f;
      this->distance = 0.0f;
    }
  }

  explicit MotionRequest_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->angle = 0.0f;
      this->distance = 0.0f;
    }
  }

  // field types and members
  using _angle_type =
    float;
  _angle_type angle;
  using _distance_type =
    float;
  _distance_type distance;

  // setters for named parameter idiom
  Type & set__angle(
    const float & _arg)
  {
    this->angle = _arg;
    return *this;
  }
  Type & set__distance(
    const float & _arg)
  {
    this->distance = _arg;
    return *this;
  }

  // constant declarations
  static const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> CMD;

  // pointer types
  using RawPtr =
    atlas_msgs::srv::MotionRequest_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const atlas_msgs::srv::MotionRequest_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<atlas_msgs::srv::MotionRequest_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<atlas_msgs::srv::MotionRequest_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      atlas_msgs::srv::MotionRequest_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<atlas_msgs::srv::MotionRequest_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      atlas_msgs::srv::MotionRequest_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<atlas_msgs::srv::MotionRequest_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<atlas_msgs::srv::MotionRequest_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<atlas_msgs::srv::MotionRequest_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__atlas_msgs__srv__MotionRequest_Request
    std::shared_ptr<atlas_msgs::srv::MotionRequest_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__atlas_msgs__srv__MotionRequest_Request
    std::shared_ptr<atlas_msgs::srv::MotionRequest_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MotionRequest_Request_ & other) const
  {
    if (this->angle != other.angle) {
      return false;
    }
    if (this->distance != other.distance) {
      return false;
    }
    return true;
  }
  bool operator!=(const MotionRequest_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MotionRequest_Request_

// alias to use template instance with default allocator
using MotionRequest_Request =
  atlas_msgs::srv::MotionRequest_Request_<std::allocator<void>>;

// constant definitions
template<typename ContainerAllocator>
const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>
MotionRequest_Request_<ContainerAllocator>::CMD = "m";

}  // namespace srv

}  // namespace atlas_msgs


#ifndef _WIN32
# define DEPRECATED__atlas_msgs__srv__MotionRequest_Response __attribute__((deprecated))
#else
# define DEPRECATED__atlas_msgs__srv__MotionRequest_Response __declspec(deprecated)
#endif

namespace atlas_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct MotionRequest_Response_
{
  using Type = MotionRequest_Response_<ContainerAllocator>;

  explicit MotionRequest_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit MotionRequest_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  // field types and members
  using _structure_needs_at_least_one_member_type =
    uint8_t;
  _structure_needs_at_least_one_member_type structure_needs_at_least_one_member;


  // constant declarations

  // pointer types
  using RawPtr =
    atlas_msgs::srv::MotionRequest_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const atlas_msgs::srv::MotionRequest_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<atlas_msgs::srv::MotionRequest_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<atlas_msgs::srv::MotionRequest_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      atlas_msgs::srv::MotionRequest_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<atlas_msgs::srv::MotionRequest_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      atlas_msgs::srv::MotionRequest_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<atlas_msgs::srv::MotionRequest_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<atlas_msgs::srv::MotionRequest_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<atlas_msgs::srv::MotionRequest_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__atlas_msgs__srv__MotionRequest_Response
    std::shared_ptr<atlas_msgs::srv::MotionRequest_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__atlas_msgs__srv__MotionRequest_Response
    std::shared_ptr<atlas_msgs::srv::MotionRequest_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MotionRequest_Response_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const MotionRequest_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MotionRequest_Response_

// alias to use template instance with default allocator
using MotionRequest_Response =
  atlas_msgs::srv::MotionRequest_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace atlas_msgs

namespace atlas_msgs
{

namespace srv
{

struct MotionRequest
{
  using Request = atlas_msgs::srv::MotionRequest_Request;
  using Response = atlas_msgs::srv::MotionRequest_Response;
};

}  // namespace srv

}  // namespace atlas_msgs

#endif  // ATLAS_MSGS__SRV__DETAIL__MOTION_REQUEST__STRUCT_HPP_
