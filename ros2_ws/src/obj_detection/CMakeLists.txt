cmake_minimum_required(VERSION 3.8)
project(obj_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(prisma_rover_interfaces REQUIRED)

add_executable(simple_node_cpp src/simple_node.cpp)
ament_target_dependencies(simple_node_cpp rclcpp)

# install the scripts
install(TARGETS
  simple_node_cpp
  DESTINATION lib/${PROJECT_NAME})

ament_python_install_package(${PROJECT_NAME})

#install python executables
install(PROGRAMS
  scripts/realsense_color_rs.py
  scripts/detect_objects_rs.py
  scripts/objects_map.py
  scripts/object_search.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
