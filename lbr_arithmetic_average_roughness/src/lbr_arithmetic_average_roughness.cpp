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
  // Subscriber
  point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/camera/camera/depth/color/points", rclcpp::SensorDataQoS(),
    std::bind(&ArithmeticAverageRoughness::pointCloudCallback, this, std::placeholders::_1));
}

ArithmeticAverageRoughness::~ArithmeticAverageRoughness()
{
  std::cout << "ArithmeticAverageRoughness class is destructed." << std::endl;
}


void ArithmeticAverageRoughness::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg){
}

}  // namespace lbr_arithmetic_average_roughness

RCLCPP_COMPONENTS_REGISTER_NODE(lbr_arithmetic_average_roughness::ArithmeticAverageRoughness)