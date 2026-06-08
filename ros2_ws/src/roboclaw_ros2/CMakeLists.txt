cmake_minimum_required(VERSION 3.8)
project(roboclaw_ros2)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(prisma_rover_interfaces REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

include_directories(
    include
)

# Export dependencies
ament_export_dependencies(
    geometry_msgs 
    nav_msgs 
    rclcpp 
    std_msgs 
    tf2 
    tf2_ros 
    tf2_geometry_msgs 
    prisma_rover_interfaces
)
ament_export_include_directories(include)

# 1. roboclaw_driver library
add_library(roboclaw_driver STATIC
    src/roboclaw_driver.cpp
    src/TimeoutSerial.cpp
)
ament_target_dependencies(roboclaw_driver
    rclcpp     
    std_msgs
    nav_msgs
)

# 2. roboclaw_roscore library
add_library(roboclaw_roscore STATIC
    src/roboclaw_roscore.cpp
)
ament_target_dependencies(roboclaw_roscore
    rclcpp
    std_msgs
    nav_msgs
    prisma_rover_interfaces
)
target_link_libraries(roboclaw_roscore
    roboclaw_driver
)

# 3. roboclaw_node executable
add_executable(roboclaw_node
    src/roboclaw_node.cpp
)
ament_target_dependencies(roboclaw_node
    rclcpp
    std_msgs
    nav_msgs
    prisma_rover_interfaces
)
target_link_libraries(roboclaw_node
    roboclaw_roscore
)

# 4. diffdrive_roscore library
add_library(diffdrive_roscore STATIC
    src/diffdrive_roscore.cpp
)
ament_target_dependencies(diffdrive_roscore
    rclcpp
    std_msgs
    nav_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    prisma_rover_interfaces
)

# 5. diffdrive_node executable
add_executable(diffdrive_node
    src/diffdrive_node.cpp
)
ament_target_dependencies(diffdrive_node
    rclcpp
    std_msgs
    nav_msgs
    prisma_rover_interfaces
)
target_link_libraries(diffdrive_node
    diffdrive_roscore
)

# Installation
install(TARGETS
    roboclaw_node
    diffdrive_node
    DESTINATION lib/${PROJECT_NAME}
)

install(FILES 
    include/TimeoutSerial.h 
    include/diffdrive_roscore.h 
    include/roboclaw_driver.h 
    include/roboclaw_roscore.h
    DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch 
    DESTINATION share/${PROJECT_NAME}/
)

ament_package()
