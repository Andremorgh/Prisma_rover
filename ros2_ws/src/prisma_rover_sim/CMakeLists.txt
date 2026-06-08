cmake_minimum_required(VERSION 3.8)
project(prisma_rover_sim)

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install launch files and worlds
install(
  DIRECTORY launch worlds models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
