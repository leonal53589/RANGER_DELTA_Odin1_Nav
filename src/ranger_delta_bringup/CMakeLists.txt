cmake_minimum_required(VERSION 3.8)
project(ranger_delta_bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

set(delta_model_dir
  "${CMAKE_CURRENT_SOURCE_DIR}/../vendor/agilex_models/ugv_gazebo_sim/ranger_delta/ranger_delta_description")
set(delta_urdf "${delta_model_dir}/urdf/ranger_delta_description.urdf")
if(NOT EXISTS "${delta_urdf}" OR NOT IS_DIRECTORY "${delta_model_dir}/meshes")
  message(FATAL_ERROR "Ranger Delta model missing. From the workspace root run: vcs import src < dependencies.repos")
endif()
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${delta_urdf}")

# Keep the upstream geometry and joints. Give its root a distinct name and
# resolve its mesh URIs through this ROS 2 package.
file(READ "${delta_urdf}" delta_description)
string(REPLACE "\"base_link\"" "\"agilex_base_link\"" delta_description "${delta_description}")
string(REPLACE "package://ranger_delta_description/" "package://${PROJECT_NAME}/agilex_delta/"
  delta_description "${delta_description}")
set(delta_install_dir "share/${PROJECT_NAME}/agilex_delta")
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/ranger_delta_description.urdf" "${delta_description}")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/ranger_delta_description.urdf"
  DESTINATION "${delta_install_dir}")
install(FILES "${delta_model_dir}/package.xml" DESTINATION "${delta_install_dir}"
  RENAME upstream-package.xml)
install(DIRECTORY "${delta_model_dir}/meshes" DESTINATION "${delta_install_dir}")

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/ranger_delta_feedback scripts/ranger_delta_command scripts/ranger_delta_teleop_gui scripts/ranger_delta_wheel_monitor scripts/ranger_delta_steer_verifier DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch urdf DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(ranger_delta_protocol test
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
endif()

ament_package()
