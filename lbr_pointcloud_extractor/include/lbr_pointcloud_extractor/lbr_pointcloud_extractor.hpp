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
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/msg/marker.hpp>
#include <vector>
#include <limits>

#include "lbr_pointcloud_extractor/visibility_control.h"

namespace lbr_pointcloud_extractor
{
  struct Point3D
  {
    float x, y, z;
  };
  struct BBox2D
  {
    int x_min, y_min, x_max, y_max;
  };

class PointCloudExtractor : public rclcpp::Node
{
public:
LBR_POINTCLOUD_EXTRACTOR_PUBLIC
  explicit PointCloudExtractor(const rclcpp::NodeOptions & options);

  virtual ~PointCloudExtractor();

private:
    /**
     * @brief This callback function runs when subscribes depth image topic.
     *
     * @param msg
     */
    void depthImageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

    /**
     * @brief This callback function runs when subscribes bounding boxes topic.
     *
     * @param msg
     */
    void bboxCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);

    /**
     * @brief This function runs when subscribes camera info topic.
     *
     * @param msg
     */
    void cameraInfoCallback(
      const sensor_msgs::msg::CameraInfo::SharedPtr msg);

    /**
     * @brief This function extract the pointcloud inside the bounding boxes
     *
     * @param bboxes, depth_img
     */
    sensor_msgs::msg::PointCloud2 extractPointCloudFromBBoxes(
      const std::vector<BBox2D> & bboxes,
      const sensor_msgs::msg::Image & depth_img);

    /**
     * @brief This function extracts the bounding box coordinate from bbox_msg
     *
     * @param bbox_msg
     */
    std::vector<BBox2D> extractBBoxCoordinates(
      const std_msgs::msg::Float32MultiArray::SharedPtr bbox_msg);

// Subscriber
rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_image_sub_;
rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr bbox_sub_;
rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

// Publisher
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr extracted_pointcloud_pub_;

// Variables
sensor_msgs::msg::PointCloud2::SharedPtr latest_pointcloud_;
sensor_msgs::msg::Image::SharedPtr latest_depth_image_;
rs2_intrinsics camera_intrinsics_;
std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
};

}  // namespace lbr_pointcloud_extractor

#endif  // LBR_POINTCLOUD_EXTRACTOR__LBR_POINTCLOUD_EXTRACTOR_HPP_