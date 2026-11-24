cmake_minimum_required(VERSION 3.8)
project(amr_description)

find_package(ament_cmake REQUIRED)

# Install the model, controller config, launch files and RViz view
install(DIRECTORY urdf config launch rviz
  DESTINATION share/${PROJECT_NAME})

# Unit tests
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_urdf test/test_urdf.py)
endif()

ament_package()
