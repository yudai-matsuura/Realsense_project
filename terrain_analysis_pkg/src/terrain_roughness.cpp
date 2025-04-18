#include terrain_analysis_pkg/terrain_rouhgness.hpp

TerrainRoughness::TerrainRoughness():Node("terrain_roughness") {
    std::cout << "TerrainRoughness class is establishes." << std::endl;

    //Subscriber
    point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/camera/camera/depth/color/points", rclcpp::SensorDataQoS(),
        std::bind(&TerrainRoughness::pointCloudCallback, this, std::placeholder::_1));
}

void TerrainRoughness::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argc);
    auto node = std::make_shared<TerrainRoughness>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}