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
constexpr size_t kBBoxElementNum = 4; // x_min, y_min, x_max, y_max
constexpr int x_min_index = 0;
constexpr int y_min_index = 1;
constexpr int x_max_index = 2;
constexpr int y_max_index = 3;
// Indices for the intrinsic camera matrix K
constexpr int K_fx = 0;
constexpr int K_cx = 2;
constexpr int K_fy = 4;
constexpr int K_cy = 5;

PointCloudExtractor::PointCloudExtractor(const rclcpp::NodeOptions & options)
: rclcpp::Node("lbr_pointcloud_extractor", options)
{
  std::cout << "PointCloudExtractor class is established." << std::endl;
  // Publisher
  extracted_pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/uneven_terrain_pointcloud", 10);

  // Subscriber
  depth_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/camera/camera/aligned_depth_to_color/image_raw", rclcpp::SensorDataQoS(),
    std::bind(&PointCloudExtractor::depthImageCallback, this, std::placeholders::_1));

  bbox_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
    "/yolo_bboxes", rclcpp::SensorDataQoS(),
    std::bind(&PointCloudExtractor::bboxCallback, this, std::placeholders::_1));

  camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "/camera/camera/aligned_depth_to_color/camera_info", rclcpp::SensorDataQoS(),
    std::bind(&PointCloudExtractor::cameraInfoCallback, this, std::placeholders::_1));
}

PointCloudExtractor::~PointCloudExtractor()
{
  std::cout << "PointCloudExtractor class is destructed." << std::endl;
}


void PointCloudExtractor::depthImageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  latest_depth_image_ = msg;
}

void PointCloudExtractor::bboxCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
#if DEBUG_ENABLED
  if (!latest_depth_image_ || camera_intrinsics_.width == 0) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Depth image or camera intrinsics not available yet. skipping callback"
    );
    return;
  }
#endif //DEBUG_ENABLED
  if (!latest_depth_image_) return;

  // ****** Get BBox's vertex coordinate ****** //
  auto bboxes = extractBBoxCoordinates(msg);

  // ****** Extract point cloud ****** //
  auto extracted_cloud = extractPointCloudFromBBoxes(bboxes, *latest_depth_image_);

  extracted_pointcloud_pub_->publish(extracted_cloud);
}

std::vector<BBox2D> PointCloudExtractor::extractBBoxCoordinates(
  const std_msgs::msg::Float32MultiArray::SharedPtr bbox_msg)
{
  std::vector<BBox2D> bboxes;
  const auto & data = bbox_msg->data;

  for (size_t i = 0; i + (kBBoxElementNum - 1) < data.size(); i += kBBoxElementNum) {
    BBox2D bbox;
    bbox.x_min = static_cast<int>(std::max(0.0f, data[i + x_min_index]));
    bbox.y_min = static_cast<int>(std::max(0.0f, data[i + y_min_index]));
    bbox.x_max = static_cast<int>(data[i + x_max_index]);
    bbox.y_max = static_cast<int>(data[i + y_max_index]);

    if (latest_depth_image_) {
      bbox.x_max = std::min(bbox.x_max, static_cast<int>(latest_depth_image_->width - 1));
      bbox.y_max = std::min(bbox.y_max, static_cast<int>(latest_depth_image_->height - 1));
    }

    bboxes.push_back(bbox);
  }
  return bboxes;
}

sensor_msgs::msg::PointCloud2 PointCloudExtractor::extractPointCloudFromBBoxes(
  const std::vector<BBox2D> & bboxes,
  const sensor_msgs::msg::Image & depth_img)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr extracted_cloud(new pcl::PointCloud<pcl::PointXYZ>);

  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvShare(std::make_shared<sensor_msgs::msg::Image>(depth_img), depth_img.encoding);
  } catch (const std::exception & e){
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    // Return an empty point cloud if conversion fails
    sensor_msgs::msg::PointCloud2 empty_cloud;
    empty_cloud.header.stamp = this->now();
    empty_cloud.header.frame_id = "camera_color_optical_frame";
    return empty_cloud;
  }
  const cv::Mat & depth_image_mat = cv_ptr->image;

  for (const auto bbox : bboxes) {
    for (int v = bbox.y_min; v <= bbox.y_max; ++v) {
      for (int u = bbox.x_min; u <= bbox.x_max; ++u) {
        uint16_t depth_mm = depth_image_mat.at<uint16_t>(v, u); // get depth
        if (depth_mm == 0) {
            continue; // No depth information
        }
        float depth_m = static_cast<float>(depth_mm) * 0.001f;

        if (depth_m > 0.0f && !std::isnan(depth_m)) {
          float pixel[2] = {static_cast<float>(u), static_cast<float>(v)};
          float point[3];
          rs2_deproject_pixel_to_point(point, &camera_intrinsics_, pixel, depth_m);

          if (std::isfinite(point[0]) && std::isfinite(point[1]) && std::isfinite(point[2])) {
            pcl::PointXYZ pcl_point(point[0], point[1], point[2]);
            extracted_cloud->points.push_back(pcl_point);
          }
        }
      }
    }
  }
  extracted_cloud->width = extracted_cloud->points.size();
  extracted_cloud->height = 1; // Unorganized point cloud
  extracted_cloud->is_dense = false;
  sensor_msgs::msg::PointCloud2 output_cloud;
  pcl::toROSMsg(*extracted_cloud, output_cloud);
  output_cloud.header.stamp = this->now();
  output_cloud.header.frame_id = "camera_color_optical_frame";

  RCLCPP_INFO(this->get_logger(), "Extracted %zu points from %zu bounding boxes",
              extracted_cloud->points.size(), bboxes.size());

  return output_cloud;
}

void PointCloudExtractor::cameraInfoCallback(
  const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  camera_intrinsics_.width = msg->width;
  camera_intrinsics_.height = msg->height;
  camera_intrinsics_.ppx = msg->k[K_cx];
  camera_intrinsics_.ppy = msg->k[K_cy];
  camera_intrinsics_.fx = msg->k[K_fx];
  camera_intrinsics_.fy = msg->k[K_fy];
  camera_intrinsics_.model = RS2_DISTORTION_NONE;
  std::copy(std::begin(msg->d), std::begin(msg->d) + 5, camera_intrinsics_.coeffs);
}

}  // namespace lbr_pointcloud_extractor

RCLCPP_COMPONENTS_REGISTER_NODE(lbr_pointcloud_extractor::PointCloudExtractor)
