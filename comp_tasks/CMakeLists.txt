cmake_minimum_required(VERSION 3.8)
project(comp_tasks)

# --------------------------------------------------
# Compiler options
# --------------------------------------------------
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --------------------------------------------------
# Dependencies
# --------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(GeographicLib REQUIRED)

find_package(yolov8_msgs REQUIRED)
find_package(comp_tasks_interfaces REQUIRED)

# --------------------------------------------------
# Install public headers
# --------------------------------------------------
install(
  DIRECTORY include/
  DESTINATION include
)

# ==================================================
# Libraries (lib/)
# ==================================================

# ---- task_lib ----
add_library(libtask_lib
  lib/task_lib.cpp
)

target_include_directories(libtask_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(libtask_lib
  rclcpp
  mavros_msgs
  geographic_msgs
  tf2
  tf2_geometry_msgs
)

# ---- bbox_calculations ----
add_library(libbbox_calculations
  lib/bbox_calculations.cpp
)

target_include_directories(libbbox_calculations
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(libbbox_calculations
  rclcpp
  geometry_msgs
  yolov8_msgs
)

# ==================================================
# Components (src/)
# ==================================================

# ---- task_component ----
add_library(task_component SHARED
  src/task_component.cpp
)

target_include_directories(task_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(task_component
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
)

rclcpp_components_register_nodes(
  task_component
  "comp_tasks::TaskComponent"
)

# ---- task_controller_component ----
add_library(task_controller_component SHARED
  src/task_controller_component.cpp
)

target_include_directories(task_controller_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(task_controller_component
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  yaml-cpp
)

rclcpp_components_register_nodes(
  task_controller_component
  "comp_tasks::TaskControllerComponent"
)

# ==================================================
# Task components (src/tasks)
# ==================================================

foreach(task home maneuvering speed)

  add_library(${task}_component SHARED
    src/tasks/${task}_component.cpp
  )

  target_include_directories(${task}_component PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  ament_target_dependencies(${task}_component
    rclcpp
    rclcpp_components
    rclcpp_lifecycle
    mavros_msgs
    geographic_msgs
    yolov8_msgs
    tf2
    tf2_geometry_msgs
    comp_tasks_interfaces
    yaml-cpp
  )

  target_link_libraries(${task}_component
    libtask_lib
    libbbox_calculations
  )

  rclcpp_components_register_nodes(
    ${task}_component
    "comp_tasks::${task^}"
  )

endforeach()

# ==================================================
# Utils components (src/utils)
# ==================================================

# ---- mavros_repub_component ----
add_library(mavros_repub_component SHARED
  src/utils/mavros_repub_component.cpp
)

target_include_directories(mavros_repub_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(mavros_repub_component
  rclcpp
  rclcpp_components
  mavros_msgs
)

rclcpp_components_register_nodes(
  mavros_repub_component
  "comp_tasks::MavROSRepublisher"
)

# ==================================================
# Standalone executables (utils)
# ==================================================

add_executable(global_to_local
  src/utils/global_to_local.cpp
)

target_include_directories(global_to_local PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_target_dependencies(global_to_local
  rclcpp
  geographic_msgs
  geometry_msgs
)

target_link_libraries(global_to_local
  GeographicLib::GeographicLib
)

add_executable(local_to_global
  src/utils/local_to_global.cpp
)

target_include_directories(local_to_global PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_target_dependencies(local_to_global
  rclcpp
  geographic_msgs
  geometry_msgs
)

target_link_libraries(local_to_global
  GeographicLib::GeographicLib
)

# ==================================================
# Export
# ==================================================
ament_export_include_directories(include)
ament_export_libraries(
  libtask_lib
  libbbox_calculations
)

ament_package()
