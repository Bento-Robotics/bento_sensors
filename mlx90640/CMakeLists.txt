
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)

# Include Cpp "include" directory
include_directories(include)

# Create Cpp executables
add_executable(mlx90640_node src/mlx90640_node.cpp)
ament_target_dependencies(mlx90640_node rclcpp)

# Install Cpp executables
install(TARGETS
  mlx90640_node
  DESTINATION lib/${PROJECT_NAME}
)
