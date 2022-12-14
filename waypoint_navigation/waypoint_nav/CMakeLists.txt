cmake_minimum_required(VERSION 3.0.2)
project(waypoint_nav)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
  move_base_msgs
  geometry_msgs
  move_base
  roscpp
  actionlib_msgs
  actionlib
  tf
  std_srvs
  dynamic_reconfigure
  base_local_planner
  dwa_local_planner
  std_msgs
)



find_package(PkgConfig)
pkg_search_module(yaml-cpp REQUIRED yaml-cpp)
pkg_search_module(waypoint_saver REQUIRED waypoint_saver)

if(NOT ${yaml-cpp_VERSION} VERSION_LESS "0.5")
add_definitions(-DNEW_YAMLCPP)
endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  DEPENDS waypoint_saver
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${yaml-cpp_INCLUDE_DIRS}
  ${waypoint_saver_INCLUDE_DIRS}
  ../waypoint_saver/include
)

add_executable(waypoint_nav src/waypoint_nav.cpp)

target_link_libraries(waypoint_nav
  ${catkin_LIBRARIES}
  ${yaml-cpp_LIBRARIES}
)

add_executable(velocity_controller src/velocity_controller.cpp)

target_link_libraries(velocity_controller
  ${catkin_LIBRARIES}
)


catkin_install_python(PROGRAMS
  scripts/tandem_run_manager.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

