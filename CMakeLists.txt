cmake_minimum_required(VERSION 3.8)
project(terrain_modelling)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)

# --- library deps: no ROS here on purpose (see below) -----------------------
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)

# --- node deps --------------------------------------------------------------
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_ros REQUIRED)

# ── library: the terrain model itself ───────────────────────────────────────
# Deliberately free of ROS so it can be exercised offline (a plain main() over
# a recorded cloud) instead of only through a live graph.
add_library(terrain_model src/terrain_model.cpp)
add_library(terrain_modelling::terrain_model ALIAS terrain_model)
target_compile_features(terrain_model PUBLIC cxx_std_17)
target_include_directories(terrain_model PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(terrain_model PUBLIC Eigen3::Eigen ${OpenCV_LIBS})
target_include_directories(terrain_model PUBLIC ${OpenCV_INCLUDE_DIRS})
target_compile_definitions(terrain_model PRIVATE "TERRAIN_MODELLING_BUILDING_LIBRARY")

# ── node: the ROS shell around the library ──────────────────────────────────
add_executable(terrain_modelling_node src/terrain_modelling_node.cpp)
target_compile_features(terrain_modelling_node PUBLIC cxx_std_17)
target_link_libraries(terrain_modelling_node terrain_model)
ament_target_dependencies(terrain_modelling_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_eigen
  pcl_conversions
  grid_map_core
  grid_map_ros
)

# ── node: acumula quadros do lidar no map_frame ──────────────────────────────
add_executable(scan_acum_node src/scan_acum.cpp)
target_compile_features(scan_acum_node PUBLIC cxx_std_17)
ament_target_dependencies(scan_acum_node
  grid_map_core   # o grid_map_core define um plugin do Eigen para todo o pacote
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
  tf2_eigen
)

# ── offline driver: enable once tools/dem_offline.cpp exists ────────────────
# Keeps iteration to seconds instead of bag + Gazebo + RViz.
option(BUILD_TOOLS "Build offline tools" OFF)
if(BUILD_TOOLS)
  add_executable(dem_offline tools/dem_offline.cpp)
  target_link_libraries(dem_offline terrain_model)
  install(TARGETS dem_offline DESTINATION lib/${PROJECT_NAME})
endif()

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(
  TARGETS terrain_model
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS terrain_modelling_node scan_acum_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(terrain_model)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(Eigen3 OpenCV)

ament_package()
