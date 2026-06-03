cmake_minimum_required(VERSION 2.8.3)
project(dog_robot_bridge)

set(CMAKE_CXX_FLAGS "-std=c++11 -O2 -Wall ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  tf
)

catkin_package(
  CATKIN_DEPENDS roscpp geometry_msgs nav_msgs sensor_msgs std_msgs tf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(dog_qnx_state_bridge
  src/dog_qnx_state_bridge.cpp
)
add_dependencies(dog_qnx_state_bridge ${catkin_EXPORTED_TARGETS})
target_link_libraries(dog_qnx_state_bridge ${catkin_LIBRARIES})

install(TARGETS dog_qnx_state_bridge
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
