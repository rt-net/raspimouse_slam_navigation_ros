cmake_minimum_required(VERSION 3.0.2)
project(raspimouse_slam)

find_package(catkin REQUIRED)
catkin_package()

install(DIRECTORY launch maps rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
