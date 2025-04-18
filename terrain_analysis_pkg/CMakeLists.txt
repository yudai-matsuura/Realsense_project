cmake_minimum_required(VERSION 3.8)
project(terrain_analysis_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpendantic)
endif()

# find dependencies
find_package(ament_cmake_REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io features filters kdtree)
find_package(pcl_conversions REQUIRED)

# Add executable
add_executable(terrain_roughness src/terrain_roughness.cpp)

# Libraries to link
ament_target_dependencies(terrain_roughness
    rclcpp
    sensor_msgs
    pcl_conversions
)

# Install 
install(TRAGETS terrain_roughness
    DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()