//
// Created by carlostojal on 01-06-2023.
//

#ifndef LOCAL_MAPPING_CORE_LOCALMAPPER_H
#define LOCAL_MAPPING_CORE_LOCALMAPPER_H

#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/point.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "lart_msgs/msg/cone_array.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_eigen/tf2_eigen.hpp"
#include <cv_bridge/cv_bridge.h>
#include <local_mapping_ros/vision/RGBCamera.h>
#include <local_mapping_ros/vision/RGBDCamera.h>
#include <local_mapping_ros/vision/ReconstructionFromDepth.h>
#include <local_mapping_ros/cnn/DAMO.h>
#include <local_mapping_ros/Utils.h>
#include <vector>
#include <set>
#include <unordered_map>
#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>

namespace t24e::local_mapper {

    /*! \brief Main class of the project. Receives images from several cameras and returns cone detections in 3D space. */
    class LocalMapper : public rclcpp::Node {

        private:
            /*! \brief Depth camera instance. */
            std::unique_ptr<local_mapper::vision::RGBDCamera> camera;

            /*! \brief Current map of cones. */
            lart_msgs::msg::ConeArray currentMap;

            /*! \brief Mutex which controls access to the cones map. */
            std::mutex mapMutex;

            /*! \brief Condition variable to control access to the cones map. */
            std::condition_variable mapCond;

            /*! \brief Map readiness flag. */
            bool mapReady = false;

            std::unique_ptr<local_mapper::cnn::DAMO> detector;

            /*! \brief Callback called whenever a new depth image is received. */
            void onDepthImage();

            /*! \brief Subscriber to the depth image topic. */
            rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depthImageSub;

            /*! \brief Subscriber to the color image topic. */
            rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr colorImageSub;

            /*! \brief Subscriber to the camera info topic. */
            rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cameraInfoSub;

            /*! \brief Publisher of the cones array. */
            rclcpp::Publisher<lart_msgs::msg::ConeArray>::SharedPtr conesPub;

            /*! \brief Publisher of the cone markers. */
            rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markersPub;

            /*! \brief Transform between the camera and the car's base frame. */
            std::unique_ptr<tf2_ros::Buffer> tfBuffer;

            /*! \brief Transform listener. */
            std::shared_ptr<tf2_ros::TransformListener> tfListener;

            /*! \brief tf timer. */
            rclcpp::TimerBase::SharedPtr tfTimer;

            /*! \brief Cone publishing timer. */
            rclcpp::TimerBase::SharedPtr conesTimer;

        public:
            LocalMapper();
            ~LocalMapper();

            /*! \brief Register a color image on a camera object. */
            void addColorImage(const cv::Mat &img);

            /*! \brief Register a depth image on a camera object. */
            void addDepthImage(const cv::Mat &img);

            /*! \brief Declare the transform between a camera and the car's base frame. */
            void setCameraTf(const Eigen::Affine3d& tf);

            /*! \brief Set the camera intrinsic matrix. */
            void setK(const Eigen::Matrix3d& K);

            /*! \brief Get the current cones map. Grabs the images from all the cameras, detects cones and
             * does 3D reconstruction. */
            lart_msgs::msg::ConeArray getCurrentMap();
    };

} // local_mapper

#endif //LOCAL_MAPPING_CORE_LOCALMAPPER_H
