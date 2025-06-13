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

#ifndef LBR_POINTCLOUD_EXTRACTOR__LBR_POINTCLOUD_EXTRACTOR_HPP_
#define LBR_POINTCLOUD_EXTRACTOR__LBR_POINTCLOUD_EXTRACTOR_HPP_

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <librealsense2/rsutil.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <vector>
#include <limits>

#include "lbr_pointcloud_extractor/visibility_control.h"

namespace lbr_pointcloud_extractor
{
  struct Point3D
  {
    float x, y, z;
  };
  struct AABB
  {
    Point3D min;
    Point3D max;
  };

class PointCloudExtractor : public rclcpp::Node
{
public:
LBR_POINTCLOUD_EXTRACTOR_PUBLIC
  explicit PointCloudExtractor(const rclcpp::NodeOptions & options);

  virtual ~PointCloudExtractor();

private:
    /**
     * @brief
     *
     * @param msg
     */
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    /**
     * @brief
     *
     * @param msg
     */
    void depthImageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

    /**
     * @brief
     *
     * @param msg
     */
    void bboxCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);

    /**
     * @brief This function get depth value at the pixel (u, v) from the depth image.
     *
     * @param depth_img
     * @param u, v
     */
    float getDepthAtPixel(
      const sensor_msgs::msg::Image & depth_img, int u, int v);

    /**
     * @brief This function separate the bbox by 4 elements.
     *
     * @param bbox_msg
     * @param depth_img
     * @param intrinsics
     */
    std::vector<AABB> generateAABBs(
      const std_msgs::msg::Float32MultiArray::SharedPtr bbox_msg,
      const sensor_msgs::msg::Image & depth_img,
      const rs2_intrinsics & intrinsics);

    /**
     * @brief This function check  if the point is inside the bounding box.
     *
     * @param pt, min_point, max_point
     */
    bool isPointInsideAABB(
      const pcl::PointXYZ & pt,
      const pcl::PointXYZ & min_point,
      const pcl::PointXYZ & max_point);

    /**
     * @brief
     *
     * @param input_cloud, aabbs
     */
    sensor_msgs::msg::PointCloud2 filterPointCloudByAABBs(
      const sensor_msgs::msg::PointCloud2 & input_cloud,
      const std::vector<AABB> & aabbs);

    /**
     * @brief
     *
     * @param msg
     */
    void cameraInfoCallback(
      const sensor_msgs::msg::CameraInfo::SharedPtr msg);

// Subscriber
rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_image_sub_;
rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr bbox_sub_;
rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

// Publisher
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr extracted_pointcloud_pub_;

// Variables
sensor_msgs::msg::PointCloud2::SharedPtr latest_pointcloud_;
sensor_msgs::msg::Image::SharedPtr latest_depth_image_;
rs2_intrinsics camera_intrinsics_;
};

}  // namespace lbr_pointcloud_extractor

#endif  // LBR_POINTCLOUD_EXTRACTOR__LBR_POINTCLOUD_EXTRACTOR_HPP_