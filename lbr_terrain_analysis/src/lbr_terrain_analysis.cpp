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

#include "lbr_terrain_analysis/lbr_terrain_analysis.hpp"

#include <rclcpp_components/register_node_macro.hpp>

#define DEBUG_ENABLED false
namespace lbr_terrain_analysis
{

TerrainAnalysis::TerrainAnalysis(const rclcpp::NodeOptions & options)
: rclcpp::Node("lbr_terrain_analysis", options)
{
  std::cout << "TerrainAnalysis class is established." << std::endl;
  // Publisher
  marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("normal_markers", 10);
  // Subscriber
  point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/camera/camera/depth/color/points", rclcpp::SensorDataQoS(),
    std::bind(&TerrainAnalysis::pointCloudCallback, this, std::placeholders::_1));

}

TerrainAnalysis::~TerrainAnalysis()
{
  std::cout << "TerrainAnalysis class is destructed." << std::endl;
}



void TerrainAnalysis::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  // Convert from ROS message to PCL
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *cloud);
  if(cloud->empty()) return;

  // Filter point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud = downsamplePointCloud(cloud);
  if(filtered_cloud->empty()) return;

  //Estimate normals
  pcl::PointCloud<pcl::Normal>::Ptr normals = estimateNormals(filtered_cloud);

  // Estimate terrain roughness
  double angle_sum = 0.0;
  double angle_sum_sq =0.0;
  int normal_count = 0;
  for (const auto& n : normals->points) {
      if(!std::isfinite(n.normal_x)) continue;
      Eigen::Vector3f normal(n.normal_x, n.normal_y, n.normal_z);
      normal.normalize();
      double angle = std::acos(normal.dot(Eigen::Vector3f::UnitZ())); // Angle between the normal vector and the Z axis
      angle_sum += angle;
      angle_sum_sq += angle * angle;
      normal_count++;
  }
  if (normal_count == 0) return;
  double mean = angle_sum / normal_count;
  double stddev = std::sqrt((angle_sum_sq / normal_count) - (mean * mean));

  RCLCPP_INFO(this->get_logger(), "Normal stddev: %.4f rad", stddev);

  // Publish to normal arrows to Rviz
  publishNormals(filtered_cloud, normals, msg->header.frame_id);

}

pcl::PointCloud<pcl::PointXYZ>::Ptr TerrainAnalysis::downsamplePointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud)
{
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud);
  sor.setLeafSize(0.02f, 0.02f, 0.02f); // 2cm voxel
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  sor.filter(*filtered_cloud);
  return filtered_cloud;

}

pcl::PointCloud<pcl::Normal>::Ptr TerrainAnalysis::estimateNormals(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud)
{
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud(cloud);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
  ne.setSearchMethod(tree);
  ne.setRadiusSearch(0.05);
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  ne.compute(*normals);
  return normals;
}

void TerrainAnalysis::publishNormals(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
  const pcl::PointCloud<pcl::Normal>::Ptr & normals,
  const std::string & frame_id)
{
  const int step = 100; // visualize every 100 normal
  int id_counter = 0;
  float scale = 0.05; // length of arrow

  for(size_t i = 0; i < cloud->points.size(); i += step) {
    if(!std::isfinite(normals->points[i].normal_x)) continue;

    geometry_msgs::msg::Point p_start, p_end;
    p_start.x = cloud->points[i].x;
    p_start.y = cloud->points[i].y;
    p_start.z = cloud->points[i].z;
    p_end.x = p_start.x + scale * normals->points[i].normal_x;
    p_end.y = p_start.y + scale * normals->points[i].normal_y;
    p_end.z = p_start.z + scale * normals->points[i].normal_z;

    visualization_msgs::msg::Marker arrow_marker;
    arrow_marker.header.frame_id = frame_id;
    arrow_marker.header.stamp = this->now();
    arrow_marker.ns = "normals";
    arrow_marker.id = id_counter++;
    arrow_marker.type = visualization_msgs::msg::Marker::ARROW;
    arrow_marker.action = visualization_msgs::msg::Marker::ADD;
    arrow_marker.scale.x = 0.01;
    arrow_marker.scale.y = 0.02;
    arrow_marker.scale.z = 0.02;
    arrow_marker.color.a = 1.0;
    arrow_marker.color.r = 0.0;
    arrow_marker.color.g = 1.0;
    arrow_marker.color.b = 0.0;
    arrow_marker.lifetime = rclcpp::Duration::from_seconds(0);

    arrow_marker.points.push_back(p_start);
    arrow_marker.points.push_back(p_end);

    marker_pub_->publish(arrow_marker);
  }

}

}  // namespace lbr_terrain_analysis

RCLCPP_COMPONENTS_REGISTER_NODE(lbr_terrain_analysis::TerrainAnalysis)
