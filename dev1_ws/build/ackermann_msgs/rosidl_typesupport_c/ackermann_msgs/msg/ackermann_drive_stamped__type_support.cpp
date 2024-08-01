// generated from rosidl_typesupport_c/resource/idl__type_support.cpp.em
// with input from ackermann_msgs:msg/AckermannDriveStamped.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "ackermann_msgs/msg/rosidl_typesupport_c__visibility_control.h"
#include "ackermann_msgs/msg/detail/ackermann_drive_stamped__struct.h"
#include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/message_type_support_dispatch.h"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_c/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace ackermann_msgs
{

namespace msg
{

namespace rosidl_typesupport_c
{

typedef struct _AckermannDriveStamped_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _AckermannDriveStamped_type_support_ids_t;

static const _AckermannDriveStamped_type_support_ids_t _AckermannDriveStamped_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _AckermannDriveStamped_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _AckermannDriveStamped_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _AckermannDriveStamped_type_support_symbol_names_t _AckermannDriveStamped_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ackermann_msgs, msg, AckermannDriveStamped)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ackermann_msgs, msg, AckermannDriveStamped)),
  }
};

typedef struct _AckermannDriveStamped_type_support_data_t
{
  void * data[2];
} _AckermannDriveStamped_type_support_data_t;

static _AckermannDriveStamped_type_support_data_t _AckermannDriveStamped_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _AckermannDriveStamped_message_typesupport_map = {
  2,
  "ackermann_msgs",
  &_AckermannDriveStamped_message_typesupport_ids.typesupport_identifier[0],
  &_AckermannDriveStamped_message_typesupport_symbol_names.symbol_name[0],
  &_AckermannDriveStamped_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t AckermannDriveStamped_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_AckermannDriveStamped_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace msg

}  // namespace ackermann_msgs

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_C_EXPORT_ackermann_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, ackermann_msgs, msg, AckermannDriveStamped)() {
  return &::ackermann_msgs::msg::rosidl_typesupport_c::AckermannDriveStamped_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
