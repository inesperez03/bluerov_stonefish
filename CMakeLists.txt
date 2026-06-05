cmake_minimum_required(VERSION 3.8)
project(bluerov_stonefish)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(stonefish_ros2 REQUIRED) 

# Asegúrate de que esté habilitado el soporte para C
enable_language(C)

# Include directories
include_directories(
  include
  ${std_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${stonefish_ros2_INCLUDE_DIRS}  # Add this line
)

# Agregar el directorio para el código Python
set(PYTHON_EXECUTABLE /usr/bin/python3)  # Asegúrate de que esta ruta sea correcta

add_executable(dvl_to_twist_node src/dvl_to_twist_node.cpp)
ament_target_dependencies(dvl_to_twist_node
  rclcpp
  geometry_msgs
  stonefish_ros2
)

add_executable(battery_status_simulated src/battery_status_simulated.cpp)
ament_target_dependencies(battery_status_simulated
  rclcpp
  sensor_msgs
  std_msgs
)

add_executable(leak_sensors_simulated src/leak_sensors_simulated.cpp)
ament_target_dependencies(leak_sensors_simulated
  rclcpp
  std_msgs
)

install(TARGETS
  dvl_to_twist_node
  battery_status_simulated
  leak_sensors_simulated
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install data and scenarios folders
install(DIRECTORY data scenarios
  DESTINATION share/${PROJECT_NAME}
)


# Enable testing and install test files
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
