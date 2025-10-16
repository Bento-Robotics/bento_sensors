cmake_minimum_required(VERSION 3.8)
project(bento_sensors)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
# find_package(<dependency> REQUIRED)


function(add_py_packages requirements_path)
  set(INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/")
  # check if pip3 is installed
  if(NOT EXISTS /usr/bin/pip3)
    message(FATAL_ERROR "pip3 is not installed! searched at /usr/bin/pip3")
    return()
  endif()
  # check if the virtual environment already exists
  if(NOT EXISTS ${INSTALL_DIR})
    message(STATUS "Install directory at ${INSTALL_DIR} does not exist, creating.")
    make_directory(${INSTALL_DIR})
  endif()
  # ensure that the given requirements.txt file exists
  if(NOT EXISTS ${requirements_path})
    message(FATAL_ERROR "Requirements file not found: ${requirements_path}")
    return()
  endif()
  # install dependencies from requirements.txt
  execute_process(
    COMMAND /usr/bin/pip3 install --no-input -t ${INSTALL_DIR} -r ${requirements_path}
    RESULT_VARIABLE pip_install_ret_code
  )
  # report error if return code is non-zero
  if(pip_install_ret_code)
    message(FATAL_ERROR "Failed to install dependencies from ${requirements_path}!")
    return()
  endif()
  # print success message
  message(STATUS "Python packages installed at ${INSTALL_DIR} with dependencies from ${requirements_path}")
endfunction()

# add subprojcts from other folders
add_subdirectory(mlx90640)
add_subdirectory(demo_sensor_py)
add_subdirectory(lcd_ssd1306)

# Install launch file
install(FILES
  bento_sensors.launch.yaml
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
