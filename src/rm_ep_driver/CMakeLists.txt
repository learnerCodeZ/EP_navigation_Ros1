cmake_minimum_required(VERSION 3.0.2)
project(rm_ep_driver)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2_ros
  std_msgs
  std_srvs
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    geometry_msgs
    nav_msgs
    sensor_msgs
    tf2_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch scripts config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
