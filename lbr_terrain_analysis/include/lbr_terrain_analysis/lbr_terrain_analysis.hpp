// Copyright (c) 2024 Tohoku Univ. Space Robotics Lab.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef lbr_terrain_analysis__lbr_terrain_analysis_HPP_
#define lbr_terrain_analysis__lbr_terrain_analysis_HPP_

#include <chrono>
#include <cstdio>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/voxel_grid.h>
#include <Eigen/Dense>
#include <cmath>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>


#include "lbr_terrain_analysis/visibility_control.h"

namespace lbr_terrain_analysis
{

class TerrainAnalysis : public rclcpp::Node
{
public:
  lbr_terrain_analysis_PUBLIC
  explicit TerrainAnalysis(const rclcpp::NodeOptions & options);

  virtual ~TerrainAnalysis();

private:
  void pointCloudCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg);


  /**
   * @brief This function filters the point cloud using a voxel grid filter.
   *
   * @param cloud
   */
  pcl::PointCloud<pcl::PointXYZ>::Ptr downsamplePointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud);

    /**
   * @brief This function filters the normals of point cloud.
   *
   * @param cloud
   */
  pcl::PointCloud<pcl::Normal>::Ptr estimateNormals(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud);

    /**
   * @brief This function visualize the arrows of normals.
   *
   * @param cloud
   * @param normals
   * @param frame_id
   */
  void publishNormals(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, const pcl::PointCloud<pcl::Normal>::Ptr & normals, const std::string & frame_id);

  // Publisher
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  // Subscriber
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
};

}  // namespace lbr_terrain_analysis

#endif  // lbr_terrain_analysis__lbr_terrain_analysis_HPP_
