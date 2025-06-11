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

};

}  // namespace lbr_pointcloud_extractor

#endif  // LBR_POINTCLOUD_EXTRACTOR__LBR_POINTCLOUD_EXTRACTOR_HPP_