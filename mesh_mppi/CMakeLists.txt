cmake_minimum_required(VERSION 3.8)
project(mesh_mppi VERSION 1.0.0)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Use the download extract timestamp with fetch content
cmake_policy(SET CMP0135 NEW)

# Options
option(BUILD_TESTING "Build tests" OFF)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(mesh_mppi_msgs REQUIRED)
find_package(mesh_map REQUIRED)
find_package(mbf_mesh_core REQUIRED)
find_package(mbf_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(LVR2 REQUIRED)

# Create object library to compile the sources.
# This allows us to build a static and a shared library while only compiling the objects once.
# The static library is used to build the tests.
add_library(
  ${PROJECT_NAME}_objects OBJECT
  src/controller/MeshMPPIBase.cpp
  src/Model.cpp
  src/scoring/CostFunction.cpp
  src/scoring/XTECostFunction.cpp
  src/Util.cpp
  src/kinematics/KinematicsBase.cpp
  src/kinematics/DifferentialDrive.cpp
  src/kinematics/Bicycle.cpp
)

# Shared libraries need Position-Independent-Code
set_property(TARGET ${PROJECT_NAME}_objects PROPERTY POSITION_INDEPENDENT_CODE 1)

target_include_directories(${PROJECT_NAME}_objects PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_objects PUBLIC lvr2::lvr2)
ament_target_dependencies(${PROJECT_NAME}_objects PUBLIC
  mesh_mppi_msgs
  mesh_map
  mbf_mesh_core
  mbf_msgs
  nav_msgs
  visualization_msgs
  ackermann_msgs
  LVR2
)

# Add shared library containing the plugin
add_library(
  ${PROJECT_NAME} SHARED
  $<TARGET_OBJECTS:${PROJECT_NAME}_objects>
  src/plugin.cpp
)

target_link_libraries(${PROJECT_NAME} PUBLIC ${PROJECT_NAME}_objects)

# Add static library for tests
add_library(
  ${PROJECT_NAME}_static
  $<TARGET_OBJECTS:${PROJECT_NAME}_objects>
)

target_link_libraries(${PROJECT_NAME}_static PUBLIC ${PROJECT_NAME}_objects)

# Install the plugin library and header files
install(DIRECTORY include DESTINATION include)
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_objects
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install the pluginlib plugin spec
pluginlib_export_plugin_description_file(mbf_mesh_core mesh_mppi.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  
  add_subdirectory(tests)
endif()

ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})
ament_package()
