cmake_minimum_required(VERSION 3.8)
project(odin_nav_sim)

find_package(ament_cmake REQUIRED)

install(PROGRAMS scripts/simulated_base.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME simulated_base)
install(DIRECTORY config launch maps rviz
  DESTINATION share/${PROJECT_NAME})

ament_package()
