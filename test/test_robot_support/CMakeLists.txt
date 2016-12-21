cmake_minimum_required(VERSION 2.8.3)

project(test_robot_support)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY meshes urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
