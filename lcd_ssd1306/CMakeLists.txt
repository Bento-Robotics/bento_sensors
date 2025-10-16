find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

add_py_packages(${CMAKE_CURRENT_LIST_DIR}/requirements.txt)

# Install Python executables
install(PROGRAMS
  ssd1306_node.py
  DESTINATION lib/${PROJECT_NAME}
)
