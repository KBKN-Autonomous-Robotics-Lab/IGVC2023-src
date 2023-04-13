################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.0.2)
project(igvc2022)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  move_base_msgs
  actionlib_msgs
  sensor_msgs
  tf2_sensor_msgs
  move_base
  actionlib
  hardware_interface
  controller_manager
  message_filters
  nodelet
  std_srvs
  fulanghua_srvs
  tf
  tf2
  tf2_ros
)

find_package(PkgConfig)
pkg_search_module(yaml-cpp REQUIRED yaml-cpp)

if(NOT ${yaml-cpp_VERSION} VERSION_LESS "0.5")
add_definitions(-DNEW_YAMLCPP)
endif()

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs message_filters nodelet sensor_msgs tf2 tf2_ros tf2_sensor_msgs
  LIBRARIES ${PROJECT_NAME}
)

################################################################################
# Build
################################################################################
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${YPSPUR_INCLUDE_DIRS}
  ${yaml-cpp_INCLUDE_DIRS}
)

add_executable(igvc_robot_driver_node src/igvc_robot_driver_node.cpp)
target_link_libraries(igvc_robot_driver_node
    ${catkin_LIBRARIES}
    ${YPSPUR_LIBRARIES}
    -lypspur
    -lypspur-md
)

add_library(pointcloud_to_laserscan src/pointcloud_to_laserscan_nodelet.cpp)
target_link_libraries(pointcloud_to_laserscan ${catkin_LIBRARIES})

add_executable(pointcloud_to_laserscan_node src/pointcloud_to_laserscan_node.cpp)
target_link_libraries(pointcloud_to_laserscan_node pointcloud_to_laserscan ${catkin_LIBRARIES})

add_executable(waypoints_saver src/waypoints_saver.cpp)
target_link_libraries(waypoints_saver ${catkin_LIBRARIES})

add_executable(control_velocity_node src/control_velocity_node.cpp)
target_link_libraries(control_velocity_node ${catkin_LIBRARIES})

add_executable(waypoints_nav src/waypoints_nav.cpp)
add_dependencies(waypoints_nav ${fulanghua_srvs_EXPORTED_TARGETS})
target_link_libraries(waypoints_nav ${catkin_LIBRARIES} ${yaml-cpp_LIBRARIES})

###############################################################################
# Install 
###############################################################################
install(FILES
    scripts/ypspur_coordinator_bridge
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
)

install(TARGETS
  pointcloud_to_laserscan
  pointcloud_to_laserscan_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
        
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES nodelets.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(roslint REQUIRED)
  roslint_cpp()
  roslint_add_test()
endif()
