cmake_minimum_required(VERSION 3.8)
project(robot_led_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetRobotLedState.srv"
)

ament_package()