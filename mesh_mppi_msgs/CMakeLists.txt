cmake_minimum_required(VERSION 3.8)
project(mesh_mppi_msgs VERSION 1.0.0)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ControlSequence.msg"
  "msg/ControlSequenceStamped.msg"
  DEPENDENCIES std_msgs
)

ament_package()
