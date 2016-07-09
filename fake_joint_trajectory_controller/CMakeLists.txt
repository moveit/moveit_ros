cmake_minimum_required(VERSION 2.8.3)
project(fake_joint_trajectory_controller)
find_package(catkin REQUIRED)
catkin_package()
catkin_install_python(PROGRAMS scripts/fake_joint_trajectory_controller
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
