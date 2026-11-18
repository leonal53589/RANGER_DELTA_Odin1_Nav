cmake_minimum_required(VERSION 3.8)
project(odin_slam)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters io)
find_package(pcl_conversions REQUIRED)

find_package(Qt5 REQUIRED COMPONENTS Core Widgets)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


file(GLOB RECORD_CLIENT_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
file(GLOB RECORD_CLIENT_HEADERS
  ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h
  ${CMAKE_CURRENT_SOURCE_DIR}/include/*.hpp
)
add_executable(odin_slam_node ${RECORD_CLIENT_SOURCES} ${RECORD_CLIENT_HEADERS})
set_target_properties(odin_slam_node PROPERTIES AUTOMOC ON)
set_target_properties(odin_slam_node PROPERTIES AUTORCC ON)

ament_target_dependencies(
  odin_slam_node
  rclcpp
  sensor_msgs
  pcl_conversions
)
target_include_directories(
  odin_slam_node
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(
  odin_slam_node
  Qt5::Core Qt5::Widgets
)

install(
  TARGETS odin_slam_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
