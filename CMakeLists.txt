cmake_minimum_required(VERSION 2.8.3)
project(robot_control)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  sensor_msgs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Build the laser_scan_publisher executable
add_executable(laser_scan_publisher src/laser_scan_publisher.cpp)

# Link against the catkin_LIBRARIES
target_link_libraries(laser_scan_publisher ${catkin_LIBRARIES})

# Mark executables and/or libraries for installation
install(TARGETS laser_scan_publisher
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


