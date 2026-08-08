cmake_minimum_required(VERSION 3.8)
project(hexapod_description)

find_package(ament_cmake REQUIRED)

# ---------------------------------------------------------------------------
# Asset installation
#
# An ament_cmake package that ships only data (no compiled code) still has to
# COPY that data into the install space. Nothing under src/ is visible at
# runtime; ROS 2 resolves package://<pkg>/... against the *share* directory of
# the install space, i.e.  <ws>/install/hexapod_description/share/hexapod_description/
#
# The EXISTS guard lets the package build cleanly while directories are still
# being added over the course of the project, instead of failing with
# "install DIRECTORY given directory that does not exist".
# ---------------------------------------------------------------------------
# NOTE ON A BUG THIS FILE USED TO HAVE
# ------------------------------------
# This block previously wrapped each install() in if(EXISTS ...). That looked
# defensive but was actively harmful, because if() is evaluated at CMake
# CONFIGURE time -- and CMake only re-configures when CMakeLists.txt itself
# changes. So a directory added AFTER the first build was invisible forever:
# the cached "false" persisted through every subsequent colcon build, and the
# symptom was "file not found in the share directory" for a file that plainly
# exists in src/. Directories are now created up front and installed
# unconditionally, which is evaluated every build.
install(
  DIRECTORY urdf meshes rviz launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
