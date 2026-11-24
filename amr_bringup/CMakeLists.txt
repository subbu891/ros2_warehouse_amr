cmake_minimum_required(VERSION 3.8)
project(amr_bringup)

find_package(ament_cmake REQUIRED)

# Install launch files and node parameters
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
