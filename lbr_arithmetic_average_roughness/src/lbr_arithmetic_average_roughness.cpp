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
  // Publisher
  marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("estimated_plane_marker", 10);
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
  // Convert from ROS message to PCL
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *cloud);
  if(cloud->empty()) return;
  // Downsample point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud = downsamplePointCloud(cloud);
  if(filtered_cloud->empty()) return;
  // Estimate plane
  Eigen::Vector4f centroid;
  Eigen::Vector3f normal;
  estimateRegressionPlane(filtered_cloud, centroid, normal);
  // Visusalize estimated plane
  publishPlaneMarker(centroid, normal, msg->header.frame_id);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr ArithmeticAverageRoughness::downsamplePointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud)
{
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud);
  sor.setLeafSize(0.02f, 0.02f, 0.02f); // 2cm voxel
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  sor.filter(*filtered_cloud);
  return filtered_cloud;
}

void ArithmeticAverageRoughness::estimateRegressionPlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, Eigen::Vector4f& plane_centroid, Eigen::Vector3f& plane_normal)
{
  // Calculate centroid of the point cloud
  pcl::compute3DCentroid(*cloud, plane_centroid);

  // Calculate deviation matrix
  Eigen::MatrixXf centered(3, cloud->size());
  for (size_t i = 0; i < cloud->size(); i++){
    centered(0, i) = cloud->points[i].x - plane_centroid[0];
    centered(1, i) = cloud->points[i].y - plane_centroid[1];
    centered(2, i) = cloud->points[i].z - plane_centroid[2];
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

void ArithmeticAverageRoughness::publishPlaneMarker(const Eigen::Vector4f& centroid, const Eigen::Vector3f& normal, const std::string & frame_id)
{
  visualization_msgs::msg::Marker plane_marker;
  plane_marker.header.frame_id = frame_id;
  plane_marker.header.stamp = this->now();
  plane_marker.ns = "estimated_plane";
  plane_marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
  plane_marker.action = visualization_msgs::msg::Marker::ADD;
  plane_marker.lifetime = rclcpp::Duration::from_seconds(0);
  plane_marker.scale.x = 1.0;
  plane_marker.scale.y = 1.0;
  plane_marker.scale.z = 1.0;
  plane_marker.color.r = 0.0f;
  plane_marker.color.g = 1.0f;
  plane_marker.color.b = 0.0f;
  plane_marker.color.a = 0.5f;

  // Create a point on a plane by generating two vectors perpendicular to the normal
  Eigen::Vector3f basis1, basis2;
  basis1 = normal.unitOrthogonal();
  basis2 = normal.cross(basis1);

  float plane_size = 0.3;
  Eigen::Vector3f center(centroid.head<3>());

  std::vector<Eigen::Vector3f> corners;
  corners.push_back(center + plane_size * ( basis1 + basis2));
  corners.push_back(center + plane_size * ( -basis1 + basis2));
  corners.push_back(center + plane_size * ( -basis1 - basis2));
  corners.push_back(center + plane_size * ( basis1 - basis2));

  // make square using two triangle
  geometry_msgs::msg::Point p0, p1, p2, p3;
  p0.x = corners[0].x(); p0.y = corners[0].y(); p0.z = corners[0].z();
  p1.x = corners[1].x(); p1.y = corners[1].y(); p1.z = corners[1].z();
  p2.x = corners[2].x(); p2.y = corners[2].y(); p2.z = corners[2].z();
  p3.x = corners[3].x(); p3.y = corners[3].y(); p3.z = corners[3].z();

  plane_marker.points.push_back(p0); plane_marker.points.push_back(p1); plane_marker.points.push_back(p2);
  plane_marker.points.push_back(p2); plane_marker.points.push_back(p3); plane_marker.points.push_back(p0);

  plane_marker.points.push_back(p2); plane_marker.points.push_back(p1); plane_marker.points.push_back(p0);
  plane_marker.points.push_back(p0); plane_marker.points.push_back(p3); plane_marker.points.push_back(p2);

  marker_pub_->publish(plane_marker);

}


// Add a function to caluculate and average the height from the plane

}  // namespace lbr_arithmetic_average_roughness

RCLCPP_COMPONENTS_REGISTER_NODE(lbr_arithmetic_average_roughness::ArithmeticAverageRoughness)