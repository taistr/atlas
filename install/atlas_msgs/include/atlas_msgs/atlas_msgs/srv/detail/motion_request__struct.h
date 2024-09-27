// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from atlas_msgs:srv/MotionRequest.idl
// generated code does not contain a copyright notice

#ifndef ATLAS_MSGS__SRV__DETAIL__MOTION_REQUEST__STRUCT_H_
#define ATLAS_MSGS__SRV__DETAIL__MOTION_REQUEST__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'CMD'.
static const char * const atlas_msgs__srv__MotionRequest_Request__CMD = "m";

/// Struct defined in srv/MotionRequest in the package atlas_msgs.
typedef struct atlas_msgs__srv__MotionRequest_Request
{
  float angle;
  float distance;
} atlas_msgs__srv__MotionRequest_Request;

// Struct for a sequence of atlas_msgs__srv__MotionRequest_Request.
typedef struct atlas_msgs__srv__MotionRequest_Request__Sequence
{
  atlas_msgs__srv__MotionRequest_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} atlas_msgs__srv__MotionRequest_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/MotionRequest in the package atlas_msgs.
typedef struct atlas_msgs__srv__MotionRequest_Response
{
  uint8_t structure_needs_at_least_one_member;
} atlas_msgs__srv__MotionRequest_Response;

// Struct for a sequence of atlas_msgs__srv__MotionRequest_Response.
typedef struct atlas_msgs__srv__MotionRequest_Response__Sequence
{
  atlas_msgs__srv__MotionRequest_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} atlas_msgs__srv__MotionRequest_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ATLAS_MSGS__SRV__DETAIL__MOTION_REQUEST__STRUCT_H_
