#ifndef TERRAIN_ANALYSIS_PKG__TERRAIN_ROUGHNESS_HPP_
#define TERRAIN_ANALYSIS_PKG__TERRAIN_ROUGHNESS_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>

#include "terrain_analysis_pkg/visibility_control.h"

namespace terrain_analysis_pkg
{

class TerrainRoughness : public rclcpp::Node
{
public:
    TERRAIN_ROUGHNESS_PUBLIC
    TerrainRoughness();

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
};

} // namespace terrain_analysis_pkg

#endif // TERRAIN_ROUGHNESS_HPP_