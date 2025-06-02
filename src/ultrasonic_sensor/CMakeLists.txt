cmake_minimum_required(VERSION 3.8)
project(ultrasonic_sensor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# --- Find libmodbus using our custom Find module ---
# Tell CMake to look for Find modules in your package's 'cmake' directory
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})

# Find libmodbus. This will now use your custom Findlibmodbus.cmake module.
# The custom module will set LIBMODBUS_INCLUDE_DIRS and LIBMODBUS_LIBRARIES.
find_package(libmodbus REQUIRED)

# --- Configuration for your custom library (UltrasonicSensor) ---
# Specify include directories for your custom headers and libmodbus headers
include_directories(
  include
  ${LIBMODBUS_INCLUDE_DIRS} # Includes from libmodbus found by pkg-config
)

# Add your custom library for the UltrasonicSensor class
# Assumes your UltrasonicSensor implementation is in src/ultrasonic_sensor_control.cpp
add_library(${PROJECT_NAME}_control STATIC
  src/ultrasonic_sensor_control.cpp
)

# Link your custom library with libmodbus
target_link_libraries(${PROJECT_NAME}_control
  PRIVATE # Keep PRIVATE here, as this is for the internal library target
    ${LIBMODBUS_LIBRARIES} # Link against libmodbus
)

# --- Configuration for your executable (sensor_distance_publisher) ---
# Define your executable
add_executable(sensor_distance_publisher
  src/ultrasonic_sensor_distance_pub.cpp
)

# Use ament_target_dependencies() to declare external ROS package dependencies.
# This ensures ROS-specific dependencies are found and propagated correctly
# for downstream packages.
ament_target_dependencies(sensor_distance_publisher
  rclcpp
  std_msgs
)

# --- CRITICAL MODIFICATION ---
# Link ALL necessary libraries to your executable in a SINGLE target_link_libraries call.
# This MUST use the PLAIN signature because ament_target_dependencies internally does.
target_link_libraries(sensor_distance_publisher
  ${PROJECT_NAME}_control # Your internal library
  ${rclcpp_LIBRARIES}     # Libraries from rclcpp (provided by find_package)
  ${std_msgs_LIBRARIES}   # Libraries from std_msgs (provided by find_package)
)
# --- END CRITICAL MODIFICATION ---

# --- Installation Rules ---
install(TARGETS
  sensor_distance_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# Install your custom library (optional, but good practice if other packages might use it)
install(TARGETS
  ${PROJECT_NAME}_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install your public headers (so other packages can find them)
install(DIRECTORY
  include/ultrasonic_sensor/
  DESTINATION include/${PROJECT_NAME}
)

ament_package()