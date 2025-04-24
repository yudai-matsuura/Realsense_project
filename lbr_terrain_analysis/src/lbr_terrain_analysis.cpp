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

  // Normal estimation
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud(cloud);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
  ne.setSearchMethod(tree);
  ne.setRadiusSearch(0.05);
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  ne.compute(*normals);

  // Estimate Terrain Analysis
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

  visualization_msg::msg::Marker marker;
  marker.header.frame_id = msg->header.frame_id;
  marker.header.stamp = this->now();
  marker.ns = "normals";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = 0.002; // width of line
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;

  for (size_t i = 0; i < cloud->point.size(); ++i) {
    if (!std::isfinite(normals->points[i].normal_x)) continue;

    geometry_msgs::msg::Point p_start, p_end;
    p_start.x = cloud->points[i].x;
    p_start.y = cloud->points[i].y;
    p_start.z = cloud->points[i].z;

    p_end.x = p_start.x + 0.05 * normals->points[i].normal_x;
    p_end.y = p_start.y + 0.05 * normals->points[i].normal_y;
    p_end.z = p_start.z + 0.05 * normals->points[i].normal_z;
  
    marker.points.push_back(p_start);
    marker.points.push_back(p_end);
  }

  marker_pub_->publish(marker);

}


}  // namespace lbr_terrain_analysis

RCLCPP_COMPONENTS_REGISTER_NODE(lbr_terrain_analysis::TerrainAnalysis)
