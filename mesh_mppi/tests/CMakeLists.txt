

##                ##
# Initialize GTest #
##                ##
include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/b514bdc898e2951020cbdca1304b75f5950d1f59.zip
)

# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

enable_testing()
include(GoogleTest)


# Add tests
add_executable(
  test_model
  src/test_model.cpp
  ../src/Util.cpp
)

add_executable(
  test_smoothing
  src/test_smoothing.cpp
)

add_executable(
  test_io
  src/test_io.cpp
)

list(
  APPEND
  TEST_TARGETS
  test_model
  test_smoothing
  test_io
)

foreach(target IN LISTS TEST_TARGETS)
  target_include_directories(${target} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
  )

  target_link_libraries(
    ${target}
    PUBLIC
    GTest::gtest_main
    ${PROJECT_NAME}_static
  )
  ament_target_dependencies(${target} PUBLIC
    mesh_map
    mbf_mesh_core
    nav_msgs
    visualization_msgs
    LVR2
  )
  gtest_discover_tests(${target}
    WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}/tests"
  )
endforeach()

