cmake_minimum_required(VERSION 3.8)
project(odin_nav_sim)

find_package(ament_cmake REQUIRED)
find_package(nav2_regulated_pure_pursuit_controller REQUIRED)
find_package(pluginlib REQUIRED)

add_library(delta_pursuit_controller SHARED src/delta_pursuit_controller.cpp)
target_compile_features(delta_pursuit_controller PUBLIC cxx_std_17)
ament_target_dependencies(delta_pursuit_controller
  nav2_regulated_pure_pursuit_controller pluginlib)
pluginlib_export_plugin_description_file(nav2_core controller_plugins.xml)
install(TARGETS delta_pursuit_controller LIBRARY DESTINATION lib)

install(PROGRAMS scripts/simulated_base.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME simulated_base)
install(PROGRAMS scripts/odin_localization.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME odin_localization)
install(FILES scripts/odin_reference_cloud.py scripts/odin_spin_guard.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch maps rviz
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(odin_localization test/test_odin_localization.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/scripts)
endif()

ament_package()
