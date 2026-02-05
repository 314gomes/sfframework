cmake_minimum_required(VERSION 3.8)
project(sfframework)

set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

# Set Open3D root directory
set(Open3D_DIR "$ENV{HOME}/Documents/open3d-devel-linux-x86_64-cxx11-abi-0.19.0/lib/cmake/Open3D")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Open3D REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)

add_executable(sfgenerator src/sfgenerator.cpp src/sfgenerator_utils.cpp)

target_link_libraries(sfgenerator
Open3D::Open3D
Eigen3::Eigen
OpenMP::OpenMP_CXX
)

ament_target_dependencies(sfgenerator
  rclcpp
  grid_map_msgs
  sensor_msgs
  laser_geometry
  grid_map_core
  grid_map_ros
  tf2_ros
)

# Build the filters plugin library
add_library(sfframework_default_filters SHARED src/default_filters.cpp)

target_link_libraries(sfframework_default_filters
  Open3D::Open3D
  Eigen3::Eigen
)

ament_target_dependencies(sfframework_default_filters
  rclcpp
  pluginlib
  grid_map_core
)

target_include_directories(sfframework_default_filters PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(sfframework_default_filters PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Build the partitioners plugin library
add_library(sfframework_default_partitioners SHARED src/default_partitioners.cpp)

target_link_libraries(sfframework_default_partitioners
  Open3D::Open3D
  Eigen3::Eigen
)

ament_target_dependencies(sfframework_default_partitioners
  rclcpp
  pluginlib
  grid_map_core
)

target_include_directories(sfframework_default_partitioners PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)


# Build the scalar field plugin library
add_library(sfframework_default_sf_strategies SHARED src/default_sf_strategies.cpp)

target_link_libraries(sfframework_default_sf_strategies
  Open3D::Open3D
  Eigen3::Eigen
  OpenMP::OpenMP_CXX
)

ament_target_dependencies(sfframework_default_sf_strategies
  rclcpp
  pluginlib
  grid_map_core
  grid_map_ros
)

target_include_directories(sfframework_default_sf_strategies PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS
  sfgenerator
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS sfframework_default_filters
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS sfframework_default_partitioners
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS sfframework_default_sf_strategies
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES sfframework_filter_plugins.xml sfframework_partitioner_plugins.xml sfframework_sf_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)
pluginlib_export_plugin_description_file(sfframework sfframework_filter_plugins.xml)
pluginlib_export_plugin_description_file(sfframework sfframework_partitioner_plugins.xml)
pluginlib_export_plugin_description_file(sfframework sfframework_sf_plugins.xml)


target_include_directories(sfgenerator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()