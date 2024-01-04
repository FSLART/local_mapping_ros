#include <rclcpp/rclcpp.hpp>
#include <local_mapping_ros/LocalMapper.h>

using namespace t24e::local_mapper;

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    rclcpp::executors::MultiThreadedExecutor executor;

    rclcpp::Node::SharedPtr node = std::make_shared<LocalMapper>();
    executor.add_node(node);

    executor.spin();

    rclcpp::shutdown();
    return 0;
}