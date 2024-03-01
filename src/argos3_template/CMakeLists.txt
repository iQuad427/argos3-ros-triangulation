cmake_minimum_required(VERSION 2.8.12)
project(argos3_template)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tri_msgs
  morpho_msgs
)

#add_message_files(
#  DIRECTORY msg
#  FILES
#)

generate_messages(
  DEPENDENCIES
  std_msgs
  tri_msgs
  morpho_msgs
)

catkin_package(
  LIBRARIES template_controller
  CATKIN_DEPENDS message_runtime roscpp rospy std_msgs
  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

link_libraries(${catkin_LIBRARIES})

# ARGoS classic part

#
# Deactivate RPATH for MacOSX
#
set(CMAKE_MACOSX_RPATH 0)

#
# Set the path where CMake will find additional scripts
#
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/cmake)

#
# Make sure we are under Unix
#
if(NOT UNIX)
  message(FATAL_ERROR "ARGoS compiles only under UNIX, sorry!")
endif(NOT UNIX)

#
# Set build options
#
include(${CMAKE_SOURCE_DIR}/cmake/ARGoSBuildOptions.cmake)

#
# Set compiler flags
#
include(${CMAKE_SOURCE_DIR}/cmake/ARGoSBuildFlags.cmake)

#
# Check for libraries
#
include(${CMAKE_SOURCE_DIR}/cmake/ARGoSBuildChecks.cmake)

#
# Set up CPack for later use
#
include(${CMAKE_SOURCE_DIR}/cmake/ARGoSPackaging.cmake)

include(${CMAKE_SOURCE_DIR}/cmake/FindPthreads.cmake)

#
# Set ARGoS include dir
#
include_directories(${CMAKE_SOURCE_DIR} ${ARGOS_INCLUDE_DIRS} ${LUA_INCLUDE_DIR})

#
# Set ARGoS link dir
#
link_directories(${ARGOS_LIBRARY_DIRS})

#
# Descend into the controllers directory
#
add_subdirectory(controllers)
#
# Descend into the loop functions directory
#
add_subdirectory(loop_functions)
