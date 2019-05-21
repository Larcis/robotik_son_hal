cmake_minimum_required(VERSION 2.8.3)
project(rtg_proje)
set(CMAKE_AUTOMOC ON)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
)

include_directories(${catkin_INCLUDE_DIRS})
catkin_package()

catkin_install_python(PROGRAMS scripts/keyboard_cmdvel.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
