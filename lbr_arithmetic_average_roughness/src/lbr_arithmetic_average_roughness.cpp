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
  // TF
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

ArithmeticAverageRoughness::~ArithmeticAverageRoughness()
{
  std::cout << "ArithmeticAverageRoughness class is destructed." << std::endl;
}


void ArithmeticAverageRoughness::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg){
  static int counter = 0;
  const int print_interval = 10; // Print every 10 messages
  const std::string target_frame = "base_link";
  const std::string source_frame = msg->header.frame_id;

  // Transform point cloud from optical frame to base frame
  geometry_msgs::msg::TransformStamped tf_msg_optical_to_base;
  try{
  tf_msg_optical_to_base = tf_buffer_->lookupTransform(target_frame, source_frame, tf2::TimePointZero, std::chrono::milliseconds(1000));
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "Could not transform point cloud from %s to %s: %s", source_frame.c_str(), target_frame.c_str(), ex.what());
  }
  sensor_msgs::msg::PointCloud2 transformed_cloud;
  tf2::doTransform(*msg, transformed_cloud, tf_msg_optical_to_base);

  // Convert from ROS message to PCL
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(transformed_cloud, *cloud);
  if(cloud->empty()) return;

  // Remove NaN values
  pcl::PointCloud<pcl::PointXYZ>::Ptr cleaned_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud, *cleaned_cloud, indices);
  if(cleaned_cloud->empty()) return;

  // Downsample point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud = downsamplePointCloud(cleaned_cloud);
  if(filtered_cloud->empty()) return;

  // Estimate plane & create heatmap
  Eigen::Vector4f centroid;
  Eigen::Vector3f normal;
  estimateRegressionPlane(filtered_cloud, centroid, normal);

  // Visualize estimated plane
  publishPlaneMarker(centroid, normal, target_frame);

  // Compute roughness score
  std::vector<float> distances = computePointToPlaneDistance(filtered_cloud, centroid, normal);
  float roughness_score = computeRoughnessScore(distances);
  if (counter % print_interval == 0) {
    std::cout << "Roughness score:" << roughness_score << std::endl;
  }
  counter++;

  // Visualize HeatMap
  publishRoughnessHeatMap(filtered_cloud, target_frame, distances);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr ArithmeticAverageRoughness::downsamplePointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud)
{
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  float voxel_size = 0.01f; // 1cm voxel size
  sor.setInputCloud(cloud);
  sor.setLeafSize(voxel_size, voxel_size, voxel_size);
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
  // --------------------------
  // Visualize estimated plane
  // --------------------------
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
  plane_marker.color.a = 0.3f;

  // Create a point on a plane by generating two vectors perpendicular to the normal
  Eigen::Vector3f basis1, basis2;
  basis1 = normal.unitOrthogonal();
  basis2 = normal.cross(basis1);

  float plane_size = 0.7;
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

  // --------------------------
  // Visualize centroid point
  // --------------------------

  visualization_msgs::msg::Marker centroid_marker;
  centroid_marker.header.frame_id = frame_id;
  centroid_marker.header.stamp = this->now();
  centroid_marker.ns = "centroid_point";
  centroid_marker.type = visualization_msgs::msg::Marker::SPHERE;
  centroid_marker.action = visualization_msgs::msg::Marker::ADD;
  centroid_marker.lifetime = rclcpp::Duration::from_seconds(0);
  centroid_marker.scale.x = 0.02;
  centroid_marker.scale.y = 0.02;
  centroid_marker.scale.z = 0.02;
  centroid_marker.color.r = 1.0f;
  centroid_marker.color.g = 0.0f;
  centroid_marker.color.b = 0.0f;
  centroid_marker.color.a = 1.0f;
  centroid_marker.pose.position.x = center.x();
  centroid_marker.pose.position.y = center.y();
  centroid_marker.pose.position.z = center.z();
  centroid_marker.pose.orientation.w = 1.0;  // No rotation
  centroid_marker.pose.orientation.x = 0.0;
  centroid_marker.pose.orientation.y = 0.0;
  centroid_marker.pose.orientation.z = 0.0;

  marker_pub_->publish(centroid_marker);
}

std::vector<float> ArithmeticAverageRoughness::computePointToPlaneDistance(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
  const Eigen::Vector4f& plane_centroid,
  const Eigen::Vector3f& plane_normal)
{
  std::vector<float> distances;
  distances.reserve(cloud->size());
  for (const auto & point : cloud->points) {
    Eigen::Vector3f vec = point.getVector3fMap() - plane_centroid.head<3>(); // vec = pi - p0
    float distance = std::abs(plane_normal.dot(vec));
    distances.push_back(distance);
  }
    return distances;
}

void ArithmeticAverageRoughness::publishRoughnessHeatMap(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
  const std::string & frame_id,
  const std::vector<float> & distances
)
{
  visualization_msgs::msg::Marker heatmap_marker;
  heatmap_marker.header.frame_id = frame_id;
  heatmap_marker.header.stamp = this->now();
  heatmap_marker.ns = "roughness_heatmap";
  heatmap_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  heatmap_marker.action = visualization_msgs::msg::Marker::ADD;
  heatmap_marker.scale.x = 0.005;
  heatmap_marker.scale.y = 0.005;
  heatmap_marker.scale.z = 0.005;
  heatmap_marker.color.a = 0.6f;
  heatmap_marker.lifetime = rclcpp::Duration::from_seconds(0);

  float max_distance = 0.0f;
  for (const float & d : distances) {
    if (d > max_distance) max_distance = d;
  }

  for (size_t i = 0; i < cloud->points.size(); ++i) {
    const auto & point = cloud->points[i];
    // Convert to ROS message
    geometry_msgs::msg::Point ros_point;
    ros_point.x = point.x;
    ros_point.y = point.y;
    ros_point.z = point.z;
    heatmap_marker.points.push_back(ros_point);

    std_msgs::msg::ColorRGBA color;
    float norm = (max_distance > 0.0f) ? distances[i] / max_distance : 0.0f; // Normalization

    // Create heatmap color
    float r = std::min(1.0f, std::max(0.0f, 1.5f - std::abs(4.0f * norm - 3.0f)));
    float g = std::min(1.0f, std::max(0.0f, 1.5f - std::abs(4.0f * norm - 2.0f)));
    float b = std::min(1.0f, std::max(0.0f, 1.5f - std::abs(4.0f * norm - 1.0f)));

    color.r = r;
    color.g = g;
    color.b = b;
    color.a = 1.0f;
    heatmap_marker.colors.push_back(color);
  }
  marker_pub_->publish(heatmap_marker);
}

float ArithmeticAverageRoughness::computeRoughnessScore(const std::vector<float> & distance){
  float sq_sum = 0.0f;
  float maximum_roughness = 0.05f; // Maximum roughness threshold in meters
  size_t filtered_pointcloud_number = distance.size();

  for (float d : distance) {
    sq_sum += d * d;
  }

  float variance = (sq_sum / filtered_pointcloud_number);
  float roughness_score = std::sqrt(variance); // Standard deviation
  float normalized_score = std::min(1.0f, roughness_score / maximum_roughness);
  return normalized_score;
}

}  // namespace lbr_arithmetic_average_roughness

RCLCPP_COMPONENTS_REGISTER_NODE(lbr_arithmetic_average_roughness::ArithmeticAverageRoughness)