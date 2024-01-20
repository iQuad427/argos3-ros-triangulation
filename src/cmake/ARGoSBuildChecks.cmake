#
# Check ARGoS
#
find_package(ARGoS REQUIRED)
include_directories(${ARGOS_INCLUDE_DIRS})
link_directories(${ARGOS_LIBRARY_DIR})
link_libraries(${ARGOS_LDFLAGS})
string(REPLACE "/lib/argos3" "" ARGOS_PREFIX "${ARGOS_LIBRARY_DIR}")
set(CMAKE_INSTALL_PREFIX ${ARGOS_PREFIX} CACHE STRING "Install path prefix, prepended onto install directories." FORCE)

#
# Find the ARGoS package
#
#if(ARGOS_BUILD_FOR_SIMULATOR)
#  find_package(PkgConfig)
#  pkg_check_modules(ARGOS REQUIRED argos3_simulator)
#  set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ARGOS_PREFIX}/share/argos3/cmake)
#  include_directories(${ARGOS_INCLUDE_DIRS})
#  link_directories(${ARGOS_LIBRARY_DIRS})
#endif(ARGOS_BUILD_FOR_SIMULATOR)

#
# Check for Qt and OpenGL when compiling for the simulator
#
if(ARGOS_BUILD_FOR_SIMULATOR)
include(FindARGoSQTOpenGL)
endif(ARGOS_BUILD_FOR_SIMULATOR)

#
# Check for Lua 5.3
#
find_package(Lua53)
if(LUA53_FOUND)
  set(ARGOS_WITH_LUA ON)
  include_directories(${LUA_INCLUDE_DIR})
endif(LUA53_FOUND)

