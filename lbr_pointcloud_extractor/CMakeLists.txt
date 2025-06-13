cmake_minimum_required(VERSION 3.8)
project(lbr_pointcloud_extractor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(realsense2 REQUIRED)
find_package(OpenCV REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_library(lbr_pointcloud_extractor src/lbr_pointcloud_extractor.cpp)
target_link_libraries(lbr_pointcloud_extractor
  realsense2::realsense2
)
rclcpp_components_register_nodes(lbr_pointcloud_extractor "lbr_pointcloud_extractor::PointCloudExtractor")

ament_auto_add_executable(lbr_pointcloud_extractor_node
  src/lbr_pointcloud_extractor_node.cpp
  src/lbr_pointcloud_extractor.cpp)

  target_link_libraries(lbr_pointcloud_extractor_node
  realsense2::realsense2
)

# install launch file
install(DIRECTORY
  DESTINATION share/${PROJECT_NAME}/
)

# gtest
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(unit_test_lbr_pointcloud_extractor test/unit_test_lbr_pointcloud_extractor.cpp)
  target_include_directories(unit_test_lbr_pointcloud_extractor PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(unit_test_lbr_pointcloud_extractor lbr_pointcloud_extractor)
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
