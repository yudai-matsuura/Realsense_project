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

#include "lbr_pointcloud_extractor/lbr_pointcloud_extractor.hpp"

#include <rclcpp_components/register_node_macro.hpp>

#define DEBUG_ENABLED false
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

PointCloudExtractor::PointCloudExtractor(const rclcpp::NodeOptions & options)
: rclcpp::Node("lbr_pointcloud_extractor", options)
{
  std::cout << "PointCloudExtractor class is established." << std::endl;
  // Publisher
  extracted_pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "pointcloud_inside_bounding_box", 10);

  // Subscriber
  pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/camera/camera/depth/color/points", rclcpp::SensorDataQoS(),
    std::bind(&PointCloudExtractor::pointCloudCallback, this, std::placeholders::_1));

  depth_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "camera/camera/depth/image_rect_raw", rclcpp::SensorDataQoS(),
    std::bind(&PointCloudExtractor::depthImageCallback, this, std::placeholders::_1));

  bbox_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
    "/bounding_boxes", rclcpp::SensorDataQoS(),
    std::bind(&PointCloudExtractor::bboxCallback, this, std::placeholders::_1));

    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "camera/camera/depth/camera_info", rclcpp::SensorDataQoS(),
      std::bind(&PointCloudExtractor::cameraInfoCallback, this, std::placeholders::_1));
}

PointCloudExtractor::~PointCloudExtractor()
{
  std::cout << "PointCloudExtractor class is destructed." << std::endl;
}


void PointCloudExtractor::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  latest_pointcloud_ = msg;
}

void PointCloudExtractor::depthImageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  latest_depth_image_ = msg;
}

void PointCloudExtractor::bboxCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  if (!latest_pointcloud_ || !latest_depth_image_) return;

  auto aabbs = generateAABBs(msg, * latest_depth_image_, camera_intrinsics_);
  auto filtered_pc = filterPointCloudByAABBs(*latest_pointcloud_, aabbs);

  extracted_pointcloud_pub_->publish(filtered_pc);
}

// TODO: Need to check how to get depth from the point in RGB
// HACK: Maybe we can use "align_depth.enable" when launch
float PointCloudExtractor::getDepth(const sensor_msgs::msg::Image & depth_img, int u, int v)
{
  const int index = v * depth_img.step + u * 2;
  uint16_t depth_raw = *(reinterpret_cast<const uint16_t*>(&depth_img.data[index]));
  return static_cast<float>(depth_raw) * 0.001f;
}

std::vector<AABB> PointCloudExtractor::generateAABBs(
  const std_msgs::msg::Float32MultiArray::SharedPtr bbox_msg,
  const sensor_msgs::msg::Image & depth_img,
  const rs2_intrinsics & intrinsics)
{
  std::vector<AABB> aabbs;
  const auto & data = bbox_msg->data;
  for (size_t i = 0; i + 3 < data.size(); i += 4) {
    float x_min = data[i];
    float y_min = data[i + 1];
    float x_max = data[i + 2];
    float y_max = data[i + 3];

    std::vector<Point3D> points_3d;

    //TODO: Need to understand here
    std::vector<std::pair<int, int>> corners = {
      {static_cast<int>(x_min), static_cast<int>(y_min)},
      {static_cast<int>(x_max), static_cast<int>(y_min)},
      {static_cast<int>(x_min), static_cast<int>(y_max)},
      {static_cast<int>(x_max), static_cast<int>(y_max)}
    };
    for(const auto & corner : corners) {
      int u = corner.first;
      int v = corner.second;
      float depth = getDepth(depth_img, u, v);
      if (depth == 0.0f || std::isnan(depth)) {
        continue;
      }

      float pixel[2] = { static_cast<float>(u), static_cast<float>(v) };
      float point[3];
      rs2_deproject_pixel_to_point(point, &intrinsics, pixel, depth);
      points_3d.push_back({ point[0], point[1], point[2] });
    }
    if (points_3d.empty()) {
      continue;
    }

    Point3D min_point = { std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max() };
    Point3D max_point = { std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest() };

    for (const auto & point : points_3d) {
      min_point.x = std::min(min_point.x, point.x);
      min_point.y = std::min(min_point.y, point.y);
      min_point.z = std::min(min_point.z, point.z);

      max_point.x = std::max(max_point.x, point.x);
      max_point.y = std::max(max_point.y, point.y);
      max_point.z = std::max(max_point.z, point.z);
    }

    aabbs.push_back({min_point, max_point});
  }

  return aabbs;
}

bool PointCloudExtractor::isPointInsideAABB(
  const pcl::PointXYZ & pt,
  const pcl::PointXYZ & min_point,
  const pcl::PointXYZ & max_point)
  {
    return  (pt.x >= min_point.x && pt.x <= max_point.x) &&
            (pt.y >= min_point.y && pt.y <= max_point.y) &&
            (pt.z >= min_point.z && pt.z <= max_point.z);
  }

sensor_msgs::msg::PointCloud2 PointCloudExtractor::filterPointCloudByAABBs(
  const sensor_msgs::msg::PointCloud2 & input_cloud,
  const std::vector<AABB> & aabbs)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(input_cloud,  *pcl_input_cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for(const auto & pt : pcl_input_cloud->points) {
      for (const auto & aabb : aabbs) {
        if(isPointInsideAABB(pt, pcl::PointXYZ(aabb.min.x, aabb.min.y, aabb.min.z), pcl::PointXYZ(aabb.max.x, aabb.max.y, aabb.max.z))) {
          pcl_output_cloud->points.push_back(pt);
          break;
        }
      }
    }

    sensor_msgs::msg::PointCloud2 output_cloud;
    pcl::toROSMsg(*pcl_output_cloud, output_cloud);
    output_cloud.header = input_cloud.header;
    return output_cloud;
  }

void PointCloudExtractor::cameraInfoCallback(
  const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    camera_intrinsics_.width = msg->width;
    camera_intrinsics_.height = msg->height;
    camera_intrinsics_.ppx = msg->k[2];
    camera_intrinsics_.ppy = msg->k[5];
    camera_intrinsics_.fx = msg->k[0];
    camera_intrinsics_.fy = msg->k[4];
    camera_intrinsics_.model = RS2_DISTORTION_NONE;
    std::copy(std::begin(msg->d), std::begin(msg->d) + 5, camera_intrinsics_.coeffs);
  }


}  // namespace lbr_pointcloud_extractor

RCLCPP_COMPONENTS_REGISTER_NODE(lbr_pointcloud_extractor::PointCloudExtractor)
