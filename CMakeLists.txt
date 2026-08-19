################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(aws_robomaker_small_warehouse_world)

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)

################################################################################
# Install
################################################################################
install(
  DIRECTORY launch models worlds maps urdf params
  DESTINATION share/${PROJECT_NAME}
)

# PROGRAMS, not FILES, so the executable bit survives; lib/, not share/, because
# that is where `ros2 run <pkg> <name>` looks.
install(
  PROGRAMS scripts/teleop scripts/ground_truth_localization
  DESTINATION lib/${PROJECT_NAME}
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/aws_robomaker_small_warehouse_world.dsv.in")

ament_export_dependencies(ament_cmake)

################################################################################
# Macro for ament package
################################################################################
ament_package()
