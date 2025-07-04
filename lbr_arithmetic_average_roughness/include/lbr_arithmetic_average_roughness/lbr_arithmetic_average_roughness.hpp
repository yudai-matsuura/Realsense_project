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
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

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

  void slopePointCloudCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  /**
   * @brief This function preprocess pointcloud.
   *
   * @param input_cloud,source_frame, target_frame
   */
  pcl::PointCloud<pcl::PointXYZ>::Ptr preProcessingPointCloud(
      const sensor_msgs::msg::PointCloud2::SharedPtr & input_cloud,
      const std::string & target_frame,
      const std::string & source_frame);

  /**
   * @brief This function transform pointcloud.
   *
   * @param input_cloud,source_frame, target_frame
   */
  sensor_msgs::msg::PointCloud2::SharedPtr transformPointCloud(
    const sensor_msgs::msg::PointCloud2::SharedPtr & input_cloud,
    const std::string & source_frame,
    const std::string & target_frame);

  /**
   * @brief This function filters the point cloud using a voxel grid filter.
   *
   * @param cloud
   */
  pcl::PointCloud<pcl::PointXYZ>::Ptr downsamplePointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud);

  /**
   * @brief This function remove outlier from point cloud.
   *
   * @param cloud
   */
  pcl::PointCloud<pcl::PointXYZ>::Ptr removeOutlierFromPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud);

  /**
   * @brief This function estimate the regression plane
   *
   * @param cloud, plane_centroid, plane_normal
   */
  void estimateRegressionPlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, Eigen::Vector4f& plane_centroid, Eigen::Vector3f& plane_normal);

  /**
   * @brief This function visualize estimated plane
   *
   * @param centroid, normal, frame_id
   */
  void publishPlaneMarker(const Eigen::Vector4f& centroid, const Eigen::Vector3f& normal, const std::string & frame_id);

  /**
   * @brief This function visualize centroid point
   *
   * @param centroid, frame_id
   */
  void publishCentroidMarker(const Eigen::Vector4f& centroid, const std::string & frame_id);

  /**
   * @brief This function calculate the avarage height from the estimated plane.
   *
   * @param cloud, centroid, normal
   */
  std::vector<float> computePointToPlaneDistance(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
                                        const Eigen::Vector4f& plane_centroid,
                                        const Eigen::Vector3f& plane_normal);

  /**
   * @brief This function get inlier indices by distance.
   *
   * @param distances, threshold
   */
  std::vector<int> getInlierIndicesByDistance(
    const std::vector<float> & distances
  );

  /**
   * @brief This function publish the HeatMap calculated from the average height.
   *
   * @param cloud, frame_id
   */
  void publishRoughnessHeatMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
    const std::string & frame_id,
    const std::vector<float> & distances);

  /**
   * @brief This function calculate the roughness score.
   *
   * @param distance
   */
  float computeRoughnessScore(const std::vector<float> & distance);

  /**
   * @brief This function calculate the estimated angle of the slope.
   *
   * @param plane_normal
   */
  float computeAngle(const Eigen::Vector3f & plane_normal);

  /**
   * @brief This function publish normal marler.
   *
   * @param normal, centroid, frame_id
   */
  void publishNormalVectorMarker(
    const Eigen::Vector4f & centroid,
    const Eigen::Vector3f & normal,
    const std::string & frame_id,
    const std::string & ns, int id, float r, float g, float b
  );


// Publisher
rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
// Subscriber
rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr uneven_terrain_pointcloud_sub_;
rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_raw_sub_;
rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr slope_pointcloud_sub_;
// Variables
std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
const int kPrintInterval_ = 10;
};

}  // namespace lbr_arithmetic_average_roughness

#endif  // LBR_ARITHMETIC_AVERAGE_ROUGHNESS__LBR_ARITHMETIC_AVERAGE_ROUGHNESS_HPP_