cmake_minimum_required(VERSION 3.8)
project(hexapod_gazebo)

find_package(ament_cmake REQUIRED)

# Unconditional: if(EXISTS) is a configure-time test and silently goes stale
# when a directory is added later. See hexapod_description/CMakeLists.txt.
install(
  DIRECTORY launch worlds config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
