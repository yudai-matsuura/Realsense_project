#include "terrain_analysis_pkg/terrain_roughness.hpp"

TerrainRoughness::TerrainRoughness():Node("terrain_roughness") {
    std::cout << "TerrainRoughness class is establishes." << std::endl;

    //Subscriber
    point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/camera/camera/depth/color/points", rclcpp::SensorDataQoS(),
        std::bind(&TerrainRoughness::pointCloudCallback, this, std::placeholder::_1));
}

void TerrainRoughness::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // Convert from ROS message to PCL
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);
    if(cloud->empty()) return;

    // Normal estimation
    pcl::NormalEstimation<pcl::PointXYZ. pcl::Normal> ne;
    ne.SetInputCloud(cloud);
    pcl::searh::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ());
    ne.setSearchMethod(tree):
    ne.setRadiusSearch(0.05);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    me.compute(*normals);

    // Estimate Terrain roughness
    double angle_sum = 0;
    double sngle_sum_sq =0;
    int normal_count = 0;
    for (const auto& n : normals->points) {
        if(!std::isfinite(n.normal_x)) continue;
        Eigen::Vector3f normal(n.normal_x, n.normal_y, n.normal_z);
        noraml.normalize();
        double angle = std::acos(normal.dot(Eigen::Vector3f::UnitZ())); // Angle between the normal vector and the Z axis
        sum += angle;
        sum_sq += angle * angle;
        count++;
    }
    if (count == 0) return;
    double mean = sum / count;
    double stddev = std::sqrt((sum_sq / count) - (mean * mean));

    RCLCPP_INFO(this->get_logger(), "Normal stddev: %.4f rad", stddev);

}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argc);
    auto node = std::make_shared<TerrainRoughness>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}