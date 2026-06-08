cmake_minimum_required(VERSION 3.8)
project(prisma_rover_navigation)

find_package(ament_cmake REQUIRED)

# Install launch files and parameters
install(
  DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
