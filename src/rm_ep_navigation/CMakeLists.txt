cmake_minimum_required(VERSION 3.0.2)
project(rm_ep_navigation)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch config rviz scripts maps
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
