#ifndef LOCAL_MAPPER_H_
#define LOCAL_MAPPER_H_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

class LocalMapper : public rclcpp::Node
{
    public:
        LocalMapper();

    private:

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_image_subscriber_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_image_subscriber_;

        void colorImageCallback(const sensor_msgs::msg::Image& msg);
        void depthImageCallback(const sensor_msgs::msg::Image& msg);
};

#endif