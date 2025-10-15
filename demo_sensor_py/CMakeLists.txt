find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)


# Install Python modules
ament_python_install_package(modules)

# Install Python executables
install(PROGRAMS
  scripts/demo_sensor_node.py
  DESTINATION lib/${PROJECT_NAME}
)
