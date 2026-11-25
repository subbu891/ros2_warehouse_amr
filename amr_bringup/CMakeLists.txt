cmake_minimum_required(VERSION 3.8)
project(amr_bringup)

find_package(ament_cmake REQUIRED)

# Install launch files and node parameters
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

# Full-stack launch test; needs Gazebo, so it only runs with -DAMR_INTEGRATION_TESTS=ON
if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  option(AMR_INTEGRATION_TESTS "Run the full-stack launch test" OFF)
  if(AMR_INTEGRATION_TESTS)
    add_launch_test(test/test_bringup_launch.py TIMEOUT 300)
  endif()
endif()

ament_package()
