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

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor exec;

  const auto lbr_pointcloud_extractor =
    std::make_shared<lbr_pointcloud_extractor::PointCloudExtractor>(rclcpp::NodeOptions());
  exec.add_node(lbr_pointcloud_extractor);

  exec.spin();

  rclcpp::shutdown();
}
