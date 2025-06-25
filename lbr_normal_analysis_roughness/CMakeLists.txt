cmake_minimum_required(VERSION 3.8)
project(lbr_normal_analysis_roughness)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS filters features)

ament_auto_find_build_dependencies()

ament_auto_add_library(lbr_normal_analysis_roughness src/lbr_normal_analysis_roughness.cpp)
rclcpp_components_register_nodes(lbr_normal_analysis_roughness "lbr_normal_analysis_roughness::TerrainAnalysis")

ament_auto_add_executable(lbr_normal_analysis_roughness_node
  src/lbr_normal_analysis_roughness_node.cpp
  src/lbr_normal_analysis_roughness.cpp)

# install launch file
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

# gtest
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(unit_test_lbr_normal_analysis_roughness test/unit_test_lbr_normal_analysis_roughness.cpp)
  target_include_directories(unit_test_lbr_normal_analysis_roughness PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(unit_test_lbr_normal_analysis_roughness lbr_normal_analysis_roughness)
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
