cmake_minimum_required(VERSION 3.0.2)
project(dog_vins_bringup)

find_package(catkin REQUIRED COMPONENTS rospy nav_msgs)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/vins_odom_adapter.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
