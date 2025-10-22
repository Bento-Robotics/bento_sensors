find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)

add_py_packages(${CMAKE_CURRENT_LIST_DIR}/requirements.txt)

# Install Python executables
install(PROGRAMS
  ina219_node.py
  DESTINATION lib/${PROJECT_NAME}
)
