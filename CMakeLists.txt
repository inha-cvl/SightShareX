cmake_minimum_required(VERSION 3.0.2)
project(sightsharex)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  message_generation
)

add_message_files(
  FILES
  ShareInfo.msg
  Path.msg
  Object.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
 LIBRARIES sightsharex
 CATKIN_DEPENDS rospy std_msgs geometry_msgs message_runtime
)

set(V2X_HEADERS
  v2x/header/crc16.h
  v2x/header/nr_v2x_interface.h
  v2x/header/v2x_app_ext.h
)

include_directories(
  v2x
  v2x/header
  ${catkin_INCLUDE_DIRS}
)

set(V2X_SOURCES
  v2x/source/crc16.c
  v2x/source/nr_v2x_interface.c
)

add_executable(sightsharex ${V2X_SOURCES})
set_source_files_properties(${V2X_SOURCES} PROPERTIES LANGUAGE CXX)
