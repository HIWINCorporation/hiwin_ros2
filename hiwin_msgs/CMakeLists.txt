cmake_minimum_required(VERSION 3.8)
project(hiwin_msgs)

find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  msg/SystemIOStates.msg
  msg/DigitalIOStates.msg
)

set(srv_files
 srv/SetIO.srv
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES geometry_msgs
)

ament_package()
