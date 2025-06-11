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

PointCloudExtractor::PointCloudExtractor(const rclcpp::NodeOptions & options)
: rclcpp::Node("lbr_pointcloud_extractor", options)
{
  std::cout << "PointCloudExtractor class is established." << std::endl;
}

PointCloudExtractor::~PointCloudExtractor()
{
  std::cout << "PointCloudExtractor class is destructed." << std::endl;
}


}  // namespace lbr_pointcloud_extractor

RCLCPP_COMPONENTS_REGISTER_NODE(lbr_pointcloud_extractor::PointCloudExtractor)
