cmake_minimum_required(VERSION 3.0.2)
project(raspimouse_slam)

find_package(catkin REQUIRED)
catkin_package()

include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
endif()