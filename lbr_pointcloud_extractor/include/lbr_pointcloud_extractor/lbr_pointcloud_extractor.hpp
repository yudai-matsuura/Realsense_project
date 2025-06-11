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
#include <std_msgs/msg/float32_multi_array.hpp>
#include <rclcpp/rclcpp.hpp>

#include "lbr_pointcloud_extractor/visibility_control.h"

namespace lbr_pointcloud_extractor
{

class PointCloudExtractor : public rclcpp::Node
{
public:
LBR_POINTCLOUD_EXTRACTOR_PUBLIC
  explicit PointCloudExtractor(const rclcpp::NodeOptions & options);

  virtual ~PointCloudExtractor();

private:
    /**
     * @brief This function separate the bbox by 4 elements.
     *
     * @param msg
     */
    void bboxCallback(
        const std_msgs::msg::Float32MultiArray::SharedPtr msg);

    /**
     * @brief This function check  if the point is inside the bounding box.
     *
     * @param u, v, bbox
     */
    bool isPointInBox(
        float u, float v, const std::array<float, 4> & bbox);

// Subscriber
rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr bbox_sub_;

// Publisher
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;

// Variables
std::vector<std::array<float, 4>> bboxes_;  // Store bounding boxes as [x_min, y_min, z_min, x_max, y_max, z_max]

};

}  // namespace lbr_pointcloud_extractor

#endif  // LBR_POINTCLOUD_EXTRACTOR__LBR_POINTCLOUD_EXTRACTOR_HPP_