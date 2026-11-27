cmake_minimum_required(VERSION 3.8)
project(odin_nav_sim)

find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_controller REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_regulated_pure_pursuit_controller REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_library(delta_pursuit_controller SHARED src/delta_pursuit_controller.cpp src/station_controller.cpp)
target_compile_features(delta_pursuit_controller PUBLIC cxx_std_17)
ament_target_dependencies(delta_pursuit_controller
  angles nav2_controller nav2_costmap_2d nav2_regulated_pure_pursuit_controller pluginlib sensor_msgs)
pluginlib_export_plugin_description_file(nav2_core controller_plugins.xml)

add_library(waypoint_segments_bt_node SHARED src/waypoint_segments.cpp)
target_compile_features(waypoint_segments_bt_node PUBLIC cxx_std_17)
target_compile_definitions(waypoint_segments_bt_node PRIVATE BT_PLUGIN_EXPORT)
ament_target_dependencies(waypoint_segments_bt_node
  behaviortree_cpp_v3 geometry_msgs nav2_util nav_msgs tf2_geometry_msgs)
add_library(obstacle_navigation_bt_node SHARED src/obstacle_navigation.cpp)
target_compile_features(obstacle_navigation_bt_node PUBLIC cxx_std_17)
target_compile_definitions(obstacle_navigation_bt_node PRIVATE BT_PLUGIN_EXPORT)
ament_target_dependencies(obstacle_navigation_bt_node
  behaviortree_cpp_v3 geometry_msgs nav2_util nav_msgs tf2_geometry_msgs)

install(TARGETS delta_pursuit_controller waypoint_segments_bt_node obstacle_navigation_bt_node LIBRARY DESTINATION lib)

install(PROGRAMS scripts/simulated_base.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME simulated_base)
install(PROGRAMS scripts/navigation_velocity_guard.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME navigation_velocity_guard)
install(PROGRAMS scripts/odin_localization.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME odin_localization)
install(PROGRAMS scripts/parking_check.py DESTINATION lib/${PROJECT_NAME} RENAME parking_check)
install(PROGRAMS scripts/station_observation.py DESTINATION lib/${PROJECT_NAME} RENAME station_observation)
install(PROGRAMS scripts/return_to_station.py DESTINATION lib/${PROJECT_NAME} RENAME return_to_station)
install(FILES scripts/station_walls.py scripts/parking_geometry.py scripts/odin_reference_cloud.py scripts/odin_spin_guard.py scripts/parking_keyframe.py
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY behavior_trees config launch maps rviz
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(station_servo test/test_station_servo.cpp)
  target_include_directories(station_servo PRIVATE src)
  target_compile_features(station_servo PRIVATE cxx_std_17)
  ament_add_gtest(terminal_parking test/test_terminal_parking.cpp)
  target_compile_features(terminal_parking PRIVATE cxx_std_17)
  ament_target_dependencies(terminal_parking nav2_costmap_2d)
  ament_add_gtest(waypoint_turn test/test_waypoint_turn.cpp)
  target_compile_features(waypoint_turn PUBLIC cxx_std_17)
  ament_target_dependencies(waypoint_turn geometry_msgs nav_msgs)
  ament_add_pytest_test(navigation_velocity_guard test/test_navigation_velocity_guard.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/scripts)
  ament_add_pytest_test(parking_keyframe test/test_parking_keyframe.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/scripts)
  ament_add_pytest_test(odin_localization test/test_odin_localization.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/scripts)
endif()

ament_package()
