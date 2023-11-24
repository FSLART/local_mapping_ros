#include <local_mapping_ros/LocalMapper.h>

LocalMapper::LocalMapper() : Node("local_mapper")
{

    this.local_mapper_ = t24e::local_mapper::LocalMapper();

    color_image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
        "color_image", 10, std::bind(&LocalMapper::colorImageCallback, this, std::placeholders::_1));
    depth_image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
        "depth_image", 10, std::bind(&LocalMapper::depthImageCallback, this, std::placeholders::_1));
}

void LocalMapper::colorImageCallback(const sensor_msgs::msg::Image& msg)
{
    RCLCPP_INFO(this->get_logger(), "Received color image");
}

void LocalMapper::depthImageCallback(const sensor_msgs::msg::Image& msg)
{
    RCLCPP_INFO(this->get_logger(), "Received depth image");
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocalMapper>());
    rclcpp::shutdown();
    return 0;
}