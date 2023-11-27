#include <rclcpp/rclcpp.hpp>
#include <local_mapping_ros/LocalMapper.h>

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<t24e::local_mapper::LocalMapper>());
    rclcpp::shutdown();
    return 0;
}