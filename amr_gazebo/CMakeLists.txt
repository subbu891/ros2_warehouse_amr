cmake_minimum_required(VERSION 3.8)
project(amr_gazebo)

find_package(ament_cmake REQUIRED)

# Install the world, layout, bridge config and launch files
install(DIRECTORY worlds config launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
