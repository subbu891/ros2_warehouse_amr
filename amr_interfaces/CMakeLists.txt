cmake_minimum_required(VERSION 3.8)
project(amr_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# Generate C++ and Python code for the custom message and action
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SafetyState.msg"
  "action/ExecuteMission.action"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
