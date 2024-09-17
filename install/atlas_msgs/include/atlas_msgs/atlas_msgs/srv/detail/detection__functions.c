// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from atlas_msgs:srv/Detection.idl
// generated code does not contain a copyright notice
#include "atlas_msgs/srv/detail/detection__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
atlas_msgs__srv__Detection_Request__init(atlas_msgs__srv__Detection_Request * msg)
{
  if (!msg) {
    return false;
  }
  // structure_needs_at_least_one_member
  return true;
}

void
atlas_msgs__srv__Detection_Request__fini(atlas_msgs__srv__Detection_Request * msg)
{
  if (!msg) {
    return;
  }
  // structure_needs_at_least_one_member
}

bool
atlas_msgs__srv__Detection_Request__are_equal(const atlas_msgs__srv__Detection_Request * lhs, const atlas_msgs__srv__Detection_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // structure_needs_at_least_one_member
  if (lhs->structure_needs_at_least_one_member != rhs->structure_needs_at_least_one_member) {
    return false;
  }
  return true;
}

bool
atlas_msgs__srv__Detection_Request__copy(
  const atlas_msgs__srv__Detection_Request * input,
  atlas_msgs__srv__Detection_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // structure_needs_at_least_one_member
  output->structure_needs_at_least_one_member = input->structure_needs_at_least_one_member;
  return true;
}

atlas_msgs__srv__Detection_Request *
atlas_msgs__srv__Detection_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  atlas_msgs__srv__Detection_Request * msg = (atlas_msgs__srv__Detection_Request *)allocator.allocate(sizeof(atlas_msgs__srv__Detection_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(atlas_msgs__srv__Detection_Request));
  bool success = atlas_msgs__srv__Detection_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
atlas_msgs__srv__Detection_Request__destroy(atlas_msgs__srv__Detection_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    atlas_msgs__srv__Detection_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
atlas_msgs__srv__Detection_Request__Sequence__init(atlas_msgs__srv__Detection_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  atlas_msgs__srv__Detection_Request * data = NULL;

  if (size) {
    data = (atlas_msgs__srv__Detection_Request *)allocator.zero_allocate(size, sizeof(atlas_msgs__srv__Detection_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = atlas_msgs__srv__Detection_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        atlas_msgs__srv__Detection_Request__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
atlas_msgs__srv__Detection_Request__Sequence__fini(atlas_msgs__srv__Detection_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      atlas_msgs__srv__Detection_Request__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

atlas_msgs__srv__Detection_Request__Sequence *
atlas_msgs__srv__Detection_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  atlas_msgs__srv__Detection_Request__Sequence * array = (atlas_msgs__srv__Detection_Request__Sequence *)allocator.allocate(sizeof(atlas_msgs__srv__Detection_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = atlas_msgs__srv__Detection_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
atlas_msgs__srv__Detection_Request__Sequence__destroy(atlas_msgs__srv__Detection_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    atlas_msgs__srv__Detection_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
atlas_msgs__srv__Detection_Request__Sequence__are_equal(const atlas_msgs__srv__Detection_Request__Sequence * lhs, const atlas_msgs__srv__Detection_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!atlas_msgs__srv__Detection_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
atlas_msgs__srv__Detection_Request__Sequence__copy(
  const atlas_msgs__srv__Detection_Request__Sequence * input,
  atlas_msgs__srv__Detection_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(atlas_msgs__srv__Detection_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    atlas_msgs__srv__Detection_Request * data =
      (atlas_msgs__srv__Detection_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!atlas_msgs__srv__Detection_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          atlas_msgs__srv__Detection_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!atlas_msgs__srv__Detection_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
atlas_msgs__srv__Detection_Response__init(atlas_msgs__srv__Detection_Response * msg)
{
  if (!msg) {
    return false;
  }
  // detection
  // angle
  // distance
  return true;
}

void
atlas_msgs__srv__Detection_Response__fini(atlas_msgs__srv__Detection_Response * msg)
{
  if (!msg) {
    return;
  }
  // detection
  // angle
  // distance
}

bool
atlas_msgs__srv__Detection_Response__are_equal(const atlas_msgs__srv__Detection_Response * lhs, const atlas_msgs__srv__Detection_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // detection
  if (lhs->detection != rhs->detection) {
    return false;
  }
  // angle
  if (lhs->angle != rhs->angle) {
    return false;
  }
  // distance
  if (lhs->distance != rhs->distance) {
    return false;
  }
  return true;
}

bool
atlas_msgs__srv__Detection_Response__copy(
  const atlas_msgs__srv__Detection_Response * input,
  atlas_msgs__srv__Detection_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // detection
  output->detection = input->detection;
  // angle
  output->angle = input->angle;
  // distance
  output->distance = input->distance;
  return true;
}

atlas_msgs__srv__Detection_Response *
atlas_msgs__srv__Detection_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  atlas_msgs__srv__Detection_Response * msg = (atlas_msgs__srv__Detection_Response *)allocator.allocate(sizeof(atlas_msgs__srv__Detection_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(atlas_msgs__srv__Detection_Response));
  bool success = atlas_msgs__srv__Detection_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
atlas_msgs__srv__Detection_Response__destroy(atlas_msgs__srv__Detection_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    atlas_msgs__srv__Detection_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
atlas_msgs__srv__Detection_Response__Sequence__init(atlas_msgs__srv__Detection_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  atlas_msgs__srv__Detection_Response * data = NULL;

  if (size) {
    data = (atlas_msgs__srv__Detection_Response *)allocator.zero_allocate(size, sizeof(atlas_msgs__srv__Detection_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = atlas_msgs__srv__Detection_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        atlas_msgs__srv__Detection_Response__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
atlas_msgs__srv__Detection_Response__Sequence__fini(atlas_msgs__srv__Detection_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      atlas_msgs__srv__Detection_Response__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

atlas_msgs__srv__Detection_Response__Sequence *
atlas_msgs__srv__Detection_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  atlas_msgs__srv__Detection_Response__Sequence * array = (atlas_msgs__srv__Detection_Response__Sequence *)allocator.allocate(sizeof(atlas_msgs__srv__Detection_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = atlas_msgs__srv__Detection_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
atlas_msgs__srv__Detection_Response__Sequence__destroy(atlas_msgs__srv__Detection_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    atlas_msgs__srv__Detection_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
atlas_msgs__srv__Detection_Response__Sequence__are_equal(const atlas_msgs__srv__Detection_Response__Sequence * lhs, const atlas_msgs__srv__Detection_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!atlas_msgs__srv__Detection_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
atlas_msgs__srv__Detection_Response__Sequence__copy(
  const atlas_msgs__srv__Detection_Response__Sequence * input,
  atlas_msgs__srv__Detection_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(atlas_msgs__srv__Detection_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    atlas_msgs__srv__Detection_Response * data =
      (atlas_msgs__srv__Detection_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!atlas_msgs__srv__Detection_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          atlas_msgs__srv__Detection_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!atlas_msgs__srv__Detection_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
