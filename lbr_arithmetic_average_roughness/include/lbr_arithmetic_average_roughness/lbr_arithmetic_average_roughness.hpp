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

#ifndef LBR_ARITHMETIC_AVERAGE_ROUGHNESS__LBR_ARITHMETIC_AVERAGE_ROUGHNESS_HPP_
#define LBR_ARITHMETIC_AVERAGE_ROUGHNESS__LBR_ARITHMETIC_AVERAGE_ROUGHNESS_HPP_

#include <chrono>
#include <cstdio>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <protocols/common/centroid.h>
#include <Eigen/Dense>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rclcpp/rclcpp.hpp>

#include "lbr_arithmetic_average_roughness/visibility_control.h"

namespace lbr_arithmetic_average_roughness
{

class ArithmeticAverageRoughness : public rclcpp::Node
{
public:
  LBR_ARITHMETIC_AVERAGE_ROUGHNESS_PUBLIC
  explicit ArithmeticAverageRoughness(const rclcpp::NodeOptions & options);

  virtual ~ArithmeticAverageRoughness();

private:
  void pointCloudCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  /**
   * @brief This function filters the point cloud using a voxel grid filter.
   *
   * @param cloud
   */
  pcl::PointCloud<pcl::PointCloudXYZ>::Ptr downsamplePointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud);

  /**
   * @brief This function estimate the regression plane
   *
   * @param cloud, plane_centroid, plane_normal
   */
  pcl::PointCloud<pcl::PointCloudXYZ>::Ptr estimateRegressionPlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, Eigen::Vector4f& plane_centroid, Eigen::Vector3f& plane_normal);

// Subscriber
rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
};

}  // namespace lbr_arithmetic_average_roughness

#endif  // LBR_ARITHMETIC_AVERAGE_ROUGHNESS__LBR_ARITHMETIC_AVERAGE_ROUGHNESS_HPP_