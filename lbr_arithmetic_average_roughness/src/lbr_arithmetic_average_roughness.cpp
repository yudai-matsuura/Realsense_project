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
constexpr float kVoxelSize = 0.01f; //[m]
constexpr int kMeanK = 50; // Number of nearest neighbors to analyze
constexpr float kStddevMulThresh = 1.5f; // Standard deviation multiplier threshold

ArithmeticAverageRoughness::ArithmeticAverageRoughness(const rclcpp::NodeOptions & options)
: rclcpp::Node("lbr_arithmetic_average_roughness", options)
{
  std::cout << "ArithmeticAverageRoughness class is established." << std::endl;
  // Publisher
  marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/marker", 10);

  // Subscriber
  // NOTE: Change the topic name to "/camera/camera/depth/color/points" if you want check with raw pointcloud
  uneven_terrain_pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/uneven_terrain_pointcloud", rclcpp::SensorDataQoS(),
    std::bind(&ArithmeticAverageRoughness::pointCloudCallback, this, std::placeholders::_1));

  slope_pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/camera/camera/depth/color/points", rclcpp::SensorDataQoS(),
    std::bind(&ArithmeticAverageRoughness::slopePointCloudCallback, this, std::placeholders::_1));

  // TF
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

ArithmeticAverageRoughness::~ArithmeticAverageRoughness()
{
  std::cout << "ArithmeticAverageRoughness class is destructed." << std::endl;
}


void ArithmeticAverageRoughness::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // ****** Preprocessing ****** //
  const std::string target_frame = "base_link";
  const std::string source_frame = "camera_color_optical_frame";
  auto filtered_cloud = preProcessingPointCloud(msg, target_frame, source_frame);
  if(!filtered_cloud) {
    RCLCPP_WARN(this->get_logger(), "Preprocessing returned a null point cloud. Skipping further processing.");
    return;
  }
  if(filtered_cloud->empty()) {
    RCLCPP_DEBUG(this->get_logger(), "Filtered point cloud is empty. Skipping further processing.");
    return;
  }

  // ****** Estimate Plane ****** //
  Eigen::Vector4f centroid;
  Eigen::Vector3f normal;
  try{
    estimateRegressionPlane(filtered_cloud, centroid, normal);
    } catch (const std::runtime_error & e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to estimate regression plane: %s", e.what());
      return;
    }

  // ****** Compute Distances ****** //
  std::vector<float> distances = computePointToPlaneDistance(filtered_cloud, centroid, normal);

  // ****** Get Inlier Indices ****** //
  std::vector<int> inlier_indices = getInlierIndicesByDistance(distances);

  pcl::PointCloud<pcl::PointXYZ>::Ptr inlier_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  for (int index : inlier_indices) {
    inlier_cloud->points.push_back(filtered_cloud->points[index]);
  }
  if (inlier_cloud->empty()) return;

  // ****** Extract Inlier Distances ****** //
  std::vector<float> inlier_distances;
  for (int index : inlier_indices) {
    inlier_distances.push_back(distances[index]);
  }

  // ****** Roughness Score ****** //
  static int counter = 0;
  float roughness_score = computeRoughnessScore(inlier_distances);
  const int kPrintInterval = 10;
  if (counter % kPrintInterval == 0) {
    std::cout << "Roughness score:" << roughness_score << std::endl;
  }
  counter++;

  // ****** Visualize ****** //
  publishPlaneMarker(centroid, normal, target_frame);
  #if DEBUG_ENABLED
  publishCentroidMarker(centroid, target_frame);
  #endif // DEBUG_ENABLED
  publishRoughnessHeatMap(inlier_cloud, target_frame, inlier_distances);
}

void ArithmeticAverageRoughness::slopePointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // ****** Preprocessing ****** //
  const std::string target_frame = "world_frame";
  const std::string source_frame = "camera_color_optical_frame";
  auto filtered_cloud = preProcessingPointCloud(msg, target_frame, source_frame);
  if(!filtered_cloud) {
    RCLCPP_WARN(this->get_logger(), "Preprocessing returned a null point cloud. Skipping further processing.");
    return;
  }
  if(filtered_cloud->empty()) {
    RCLCPP_DEBUG(this->get_logger(), "Filtered point cloud is empty. Skipping further processing.");
    return;
  }

  // ****** Estimate Plane ****** //
  Eigen::Vector4f centroid;
  Eigen::Vector3f normal;
  try{
  estimateRegressionPlane(filtered_cloud, centroid, normal);
  } catch (const std::runtime_error & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to estimate regression plane: %s", e.what());
    return;
  }

  // ****** Publish Local Normal Marker ****** //
  publishNormalVectorMarker(
    centroid, normal, target_frame, "world_normal", 0, 1.0f, 1.0f, 0.0f);

  // ****** Compute Angle ****** //
  float angle = computeAngle(normal);
  RCLCPP_INFO(this->get_logger(), "Inclination angle [deg]: %.2f", angle);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr ArithmeticAverageRoughness::preProcessingPointCloud(
  const sensor_msgs::msg::PointCloud2::SharedPtr & input_cloud,
  const std::string & target_frame,
  const std::string & source_frame)
{
  // ****** Transform ****** //
  auto transformed_cloud = transformPointCloud(input_cloud, target_frame, source_frame);
  if (!transformed_cloud) return nullptr;

  // ****** Convert Message Type ****** //
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*transformed_cloud, *cloud);
  if(cloud->empty()) return nullptr;

  // ****** Filtering ****** //
  pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud = downsamplePointCloud(cloud);
  if(downsampled_cloud->empty()) return nullptr;
  pcl::PointCloud<pcl::PointXYZ>::Ptr nan_removed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*downsampled_cloud, *nan_removed_cloud, indices);
  if(nan_removed_cloud->empty()) return nullptr;
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud = removeOutlierFromPointCloud(nan_removed_cloud);
  if(filtered_cloud->empty()) return nullptr;

  return filtered_cloud;
}

sensor_msgs::msg::PointCloud2::SharedPtr ArithmeticAverageRoughness::transformPointCloud(
  const sensor_msgs::msg::PointCloud2::SharedPtr & input_cloud,
  const std::string & target_frame, const std::string & source_frame)
{
  geometry_msgs::msg::TransformStamped tf_msg;
  try{
    tf_msg = tf_buffer_->lookupTransform(target_frame, source_frame, tf2::TimePointZero, std::chrono::milliseconds(100));
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "Could not transform point cloud from %s to %s: %s", source_frame.c_str(), target_frame.c_str(), ex.what());
      return nullptr;
    }
    auto transformed = std::make_shared<sensor_msgs::msg::PointCloud2>();
    tf2::doTransform(*input_cloud, *transformed, tf_msg);
    return transformed;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr ArithmeticAverageRoughness::downsamplePointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud)
{
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud);
  sor.setLeafSize(kVoxelSize, kVoxelSize, kVoxelSize);
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  sor.filter(*filtered_cloud);
  return filtered_cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr ArithmeticAverageRoughness::removeOutlierFromPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud)
{
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud);
  sor.setMeanK(kMeanK);
  sor.setStddevMulThresh(kStddevMulThresh);
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
  // Calculate the covariance matrix
  Eigen::Matrix3f covariance = centered * centered.transpose();

  // Calculates the eigenvalues of the covariance matrix
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance);
  if (solver.info() != Eigen::Success) {
    throw std::runtime_error("Eigen decomposition failed");
  }
  // The eigenvector corresponding to the smallest eigenvalue (normal of the plane)
  //TODO: Need to consider the direction of the normal vector
  plane_normal = solver.eigenvectors().col(0);
}

void ArithmeticAverageRoughness::publishPlaneMarker(const Eigen::Vector4f & centroid, const Eigen::Vector3f & normal, const std::string & frame_id)
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
  plane_marker.color.a = 0.3f;
  // Create a point on a plane by generating two vectors perpendicular to the normal
  Eigen::Vector3f basis1, basis2;
  basis1 = normal.unitOrthogonal();
  basis2 = normal.cross(basis1);
  const float kPlaneSize = 0.7f;
  Eigen::Vector3f center(centroid.head<3>());

  std::vector<Eigen::Vector3f> corners;
  corners.push_back(center + kPlaneSize * ( basis1 + basis2));
  corners.push_back(center + kPlaneSize * ( -basis1 + basis2));
  corners.push_back(center + kPlaneSize * ( -basis1 - basis2));
  corners.push_back(center + kPlaneSize * ( basis1 - basis2));

  // make square using two triangle
  geometry_msgs::msg::Point p0, p1, p2, p3;
  p0.x = corners[0].x(); p0.y = corners[0].y(); p0.z = corners[0].z();
  p1.x = corners[1].x(); p1.y = corners[1].y(); p1.z = corners[1].z();
  p2.x = corners[2].x(); p2.y = corners[2].y(); p2.z = corners[2].z();
  p3.x = corners[3].x(); p3.y = corners[3].y(); p3.z = corners[3].z();

  // Both side of the plane
  plane_marker.points.push_back(p0); plane_marker.points.push_back(p1); plane_marker.points.push_back(p2);
  plane_marker.points.push_back(p2); plane_marker.points.push_back(p3); plane_marker.points.push_back(p0);
  plane_marker.points.push_back(p2); plane_marker.points.push_back(p1); plane_marker.points.push_back(p0);
  plane_marker.points.push_back(p0); plane_marker.points.push_back(p3); plane_marker.points.push_back(p2);

  marker_pub_->publish(plane_marker);
}

#if DEBUG_ENABLED
void ArithmeticAverageRoughness::publishCentroidMarker(const Eigen::Vector4f & centroid, const std::string & frame_id)
{
  Eigen::Vector3f center(centroid.head<3>());
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
#endif // DEBUG_ENABLED

void ArithmeticAverageRoughness::publishNormalVectorMarker(
  const Eigen::Vector4f & centroid, const Eigen::Vector3f & normal,
  const std::string & frame_id, const std::string & ns, int id,
  float r, float g, float b)
{
  visualization_msgs::msg::Marker arrow_marker;
  arrow_marker.header.frame_id = frame_id;
  arrow_marker.header.stamp = this->now();
  arrow_marker.ns = ns;
  arrow_marker.id = id;
  arrow_marker.type = visualization_msgs::msg::Marker::ARROW;
  arrow_marker.action = visualization_msgs::msg::Marker::ADD;
  arrow_marker.lifetime = rclcpp::Duration::from_seconds(0);

  geometry_msgs::msg::Point start_point, end_point;
  start_point.x = centroid[0];
  start_point.y = centroid[1];
  start_point.z = centroid[2];
  const float kArrowLength = 0.3f;
  Eigen::Vector3f normal_normalized = normal.normalized();
  end_point.x = start_point.x + normal_normalized.x() * kArrowLength;
  end_point.y = start_point.y + normal_normalized.y() * kArrowLength;
  end_point.z = start_point.z + normal_normalized.z() * kArrowLength;

  arrow_marker.points.push_back(start_point);
  arrow_marker.points.push_back(end_point);
  arrow_marker.scale.x = 0.01;
  arrow_marker.scale.y = 0.02;
  arrow_marker.scale.z = 0.05;
  arrow_marker.color.r = r;
  arrow_marker.color.g = g;
  arrow_marker.color.b = b;
  arrow_marker.color.a = 0.9f;

  marker_pub_->publish(arrow_marker);
}

std::vector<float> ArithmeticAverageRoughness::computePointToPlaneDistance(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
  const Eigen::Vector4f& plane_centroid,
  const Eigen::Vector3f& plane_normal)
{
  std::vector<float> distances;
  distances.reserve(cloud->size());
  Eigen::Vector3f center(plane_centroid.head<3>());
  for (const auto & point : cloud->points) {
    Eigen::Vector3f centroid_to_point = point.getVector3fMap() - center;
    float distance = std::abs(plane_normal.dot(centroid_to_point));
    distances.push_back(distance);
  }
    return distances;
}

std::vector<int> ArithmeticAverageRoughness::getInlierIndicesByDistance(
  const std::vector<float> & distances)
{
  std::vector<int> indices;
  const float kDistanceThreshold = 0.2f; //[m]
  for (size_t i = 0; i < distances.size(); ++i) {
    if (distances[i] < kDistanceThreshold) {
      indices.push_back(i);
    }
  }
  return indices;
}

void ArithmeticAverageRoughness::publishRoughnessHeatMap(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
  const std::string & frame_id,
  const std::vector<float> & distances)
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
  for (const float & point_distance : distances) {
    if (point_distance > max_distance) max_distance = point_distance;
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

float ArithmeticAverageRoughness::computeRoughnessScore(const std::vector<float> & distances)
{
  float sq_sum = 0.0f;
  // TODO: This kMaximumRoughness value needs to be adjusted
  float kMaximumRoughness = 0.05f; // Maximum roughness threshold in meters
  size_t filtered_pointcloud_number = distances.size();

  for (float point_distance : distances) {
    sq_sum += point_distance * point_distance;
  }

  float roughness_score = std::sqrt(sq_sum / filtered_pointcloud_number); // Standard deviation
  float normalized_score = std::min(1.0f, roughness_score / kMaximumRoughness);
  return normalized_score;
}

float ArithmeticAverageRoughness::computeAngle(
  const Eigen::Vector3f & plane_normal)
{
  Eigen::Vector3f world_z_axis(0.0f, 0.0f, 1.0f);
  float inner_product = plane_normal.normalized().dot(world_z_axis);
  float angle_rad = std::acos(std::clamp(inner_product, -1.0f, 1.0f));
  float angle = angle_rad * 180.0f / static_cast<float>(M_PI); // Convert to degrees
  if (angle > 90.0f) {
    angle = 180.0f - angle;
  }
  return angle;
}

}  // namespace lbr_arithmetic_average_roughness

RCLCPP_COMPONENTS_REGISTER_NODE(lbr_arithmetic_average_roughness::ArithmeticAverageRoughness)