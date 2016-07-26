cmake_minimum_required(VERSION 2.8.12)
project(moveit_ros_tests)

find_package(catkin)
catkin_package()

if (CATKIN_ENABLE_TESTING)
  find_package(rostest)
  add_rostest(scene_sync.test)
endif()
