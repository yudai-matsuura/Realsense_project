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

#include "lbr_arithmetic_average_roughness/lbr_arithmetic_average_roughness.hpp"

#include <rclcpp_components/register_node_macro.hpp>

#define DEBUG_ENABLED false
namespace lbr_arithmetic_average_roughness
{

ArithmeticAverageRoughness::ArithmeticAverageRoughness(const rclcpp::NodeOptions & options)
: rclcpp::Node("lbr_arithmetic_average_roughness", options)
{
  std::cout << "ArithmeticAverageRoughness class is established." << std::endl;
  // Subscriber
  point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/camera/camera/depth/color/points", rclcpp::SensorDataQoS(),
    std::bind(&ArithmeticAverageRoughness::pointCloudCallback, this, std::placeholders::_1));
}

ArithmeticAverageRoughness::~ArithmeticAverageRoughness()
{
  std::cout << "ArithmeticAverageRoughness class is destructed." << std::endl;
}


void ArithmeticAverageRoughness::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg){
  // まず点群にフィルターをかける
  // 平面推定
  // 可視化
}

pcl::PointCloud<pcl::PointCloudXYZ>::Ptr ArithmeticAverageRoughness::downsamplePointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud)
{
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud);
  sor.setLeafSize(0.02f, 0.02f, 0.02f); // 2cm voxel
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  sor.filter(*filtered_cloud);
  return filtered_cloud;
}

// Add a function to estimate the regression plane 

void ArithmeticAverageRoughness::estimateRegressionPlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, Eigen::Vector4f& plane_centroid, Eigen::Vector3f& plane_normal)
{
  // Calculate centroid of the point cloud
  pcl::compute3Dcentroid(*cloud, plane_centroid);

  // Calculate deviation matrix
  Eigen::MatrixXf centered(3, cloud->size());
  for (size_t i = 0; i < cloud->size(); i++){
    centered(0, i) = cloud->points[i].x - plane=centroid[0];
    centered(1, i) = cloud->points[i].y - plane=centroid[1];
    centered(2, i) = cloud->points[i].z - plane=centroid[2];
  }

  // Calculate the convariance matrix
  Eigen::Matrix3f convariance = centered * centered.transpose();

  // Calculates the eigenvalues of the convariance matrix
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(convariance);
  if (solver.info() != Eigen::Success) {
    throw std::runtime_error("Eigen decomposition failed");
  }

  // The eigenvector corresponding to the smallest eigenvalue (normal of the plane)
  plane_normal = solver.eigenvectors().col(0);

}


// Add a function to caluculate and average the height from the plane

}  // namespace lbr_arithmetic_average_roughness

RCLCPP_COMPONENTS_REGISTER_NODE(lbr_arithmetic_average_roughness::ArithmeticAverageRoughness)