//
// Created by carlostojal on 01-06-2023.
//

#include <local_mapping_ros/LocalMapper.h>

#define CONE_DETECTOR_FILE_PATH "models/DAMO.pth"

namespace t24e {
    namespace local_mapper {
        LocalMapper::LocalMapper() : Node("local_mapper") {
            // instantiate the camera
            this->camera = std::make_unique<local_mapper::vision::RGBDCamera>();

            std::string path(CONE_DETECTOR_FILE_PATH);

            // instantiate and initialize the cone detector
            this->detector = std::make_unique<local_mapper::cnn::DAMO>(path);
            this->detector->init();

            // subscribe to the depth image topic
            this->depthImageSub = this->create_subscription<sensor_msgs::msg::Image>(
                    "/camera/depth/image_raw",
                    10,
                    [this](const sensor_msgs::msg::Image::SharedPtr msg) {
                        // convert the ROS image to a cv::Mat
                        cv_bridge::CvImagePtr cv_ptr;
                        try {
                            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
                        } catch (cv_bridge::Exception &e) {
                            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "cv_bridge exception: %s", e.what());
                            return;
                        }

                        // add the image to the camera
                        this->addDepthImage(cv_ptr->image);
                    });

            // subscribe to the color image topic
            this->colorImageSub = this->create_subscription<sensor_msgs::msg::Image>(
                    "/camera/color/image_raw",
                    10,
                    [this](const sensor_msgs::msg::Image::SharedPtr msg) {
                        // convert the ROS image to a cv::Mat
                        cv_bridge::CvImagePtr cv_ptr;
                        try {
                            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
                        } catch (cv_bridge::Exception &e) {
                            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "cv_bridge exception: %s", e.what());
                            return;
                        }

                        // add the image to the camera
                        this->addColorImage(cv_ptr->image);
                    });

            // subscribe to camerainfo topic
            this->cameraInfoSub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
                    "/camera/depth/camera_info",
                    10,
                    [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
                        // convert the ROS camera info to an Eigen matrix
                        Eigen::Matrix3d K;
                        K << msg->k[0], msg->k[1], msg->k[2],
                                msg->k[3], msg->k[4], msg->k[5],
                                msg->k[6], msg->k[7], msg->k[8];

                        // set the camera's intrinsic matrix
                        this->setK(K);
                    });


            // initialize the cones publisher
            this->conesPub = this->create_publisher<lart_msgs::msg::ConeArray>(
                    "/cones",
                    10);

            // initialize the markers publisher
            this->markersPub = this->create_publisher<visualization_msgs::msg::MarkerArray>(
                    "/markers",
                    10);

            // initialize the tf buffer and listener
            this->tfBuffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            this->tfListener = std::make_shared<tf2_ros::TransformListener>(*this->tfBuffer);

            // create the tf timer to update the tf
            this->tfTimer = this->create_wall_timer(
                    std::chrono::milliseconds(100),
                    [this]() {
                        // get the transform from the camera to the car's base frame
                        geometry_msgs::msg::TransformStamped tf;
                        try {
                            tf = this->tfBuffer->lookupTransform("base_link", "camera_link", tf2::TimePointZero);
                        } catch (const tf2::TransformException &ex) {
                            RCLCPP_WARN(this->get_logger(), "%s", ex.what());
                            return;
                        }

                        // convert the transform to an Eigen matrix
                        Eigen::Affine3d tfEigen;
                        tfEigen.matrix() << tf.transform.rotation.w, tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z,
                                tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z, 1;

                        // set the camera's transform
                        this->setCameraTf(tfEigen);
                    });

            this->conesTimer = this->create_wall_timer(
                std::chrono::milliseconds(50),
                [this]() {

                    // get the current map
                    // this is the map consumer, so it will idle waiting for a new map using a condition variable
                    lart_msgs::msg::ConeArray map = this->getCurrentMap();

                    // initialize the marker array
                    visualization_msgs::msg::MarkerArray markers;
                    for(auto c : map.cones) {

                        // get the color
                        std::vector<std::uint8_t> color = local_mapper::Utils::getColorByLabel((std::int16_t) c.class_type.data);

                        visualization_msgs::msg::Marker marker;
                        marker.header.frame_id = "base_link";
                        marker.header.stamp = this->get_clock()->now();
                        marker.ns = "cones";
                        marker.type = visualization_msgs::msg::Marker::CYLINDER;
                        marker.action = visualization_msgs::msg::Marker::ADD;
                        marker.pose.position = c.position;
                        marker.scale.x = 0.1;
                        marker.scale.y = 0.1;
                        marker.scale.z = 0.2;
                        marker.color.a = 1.0;
                        marker.color.r = color[0];
                        marker.color.g = color[1];
                        marker.color.b = color[2];
                        markers.markers.push_back(marker);
                    }
                    // publish the markers
                    this->markersPub->publish(markers);
                    
                    // publish the cones
                    this->conesPub->publish(map);
                }
            );

        }

        LocalMapper::~LocalMapper() {
            // explicitly delete the camera pointer, to be sure
            this->camera.reset();

            // explicitly delete the detector instance
            this->detector.reset();
        }

        void LocalMapper::onDepthImage() {

            // this method is basically the map producer

            lart_msgs::msg::ConeArray newMap;

            cv::Mat img = this->camera->getLastDepthImage().second;

            // detect cones on the last color image
            std::vector<local_mapper::cnn::bounding_box_t> detectedCones = this->detector->detectCones(img);

            // iterate the detected cones
            for(auto c : detectedCones) {
                // find the centroid
                int xCoord = c.box.first.first + (c.box.second.first / 2);
                int yCoord = c.box.first.second + (c.box.second.second / 2);
                Eigen::Vector3d pixel(xCoord, yCoord, img.at<int>(xCoord,yCoord));

                // reconstruct the cone to a point
                geometry_msgs::msg::Point point = local_mapper::vision::ReconstructionFromDepth::deprojectPixelToPoint(*this->camera, pixel);

                // create the cone message
                lart_msgs::msg::Cone cone;
                cone.position = point;
                cone.class_type.data = (std::int16_t) c.label;

                // add the cone to the array
                newMap.cones.push_back(cone);
            }

            // update the map object
            std::unique_lock lock(this->mapMutex);
            this->currentMap = newMap;

            this->mapReady = true;
            lock.unlock();

            // notify waiting thread that a new map is ready
            this->mapCond.notify_one();
        }
        
        void LocalMapper::addColorImage(const cv::Mat &img) {

            this->camera->captureColorImage(img);
        }

        void LocalMapper::addDepthImage(const cv::Mat &img) {

            this->camera->captureDepthImage(img);
            this->onDepthImage();
        }

        void LocalMapper::setCameraTf(const Eigen::Affine3d& tf) {

            this->camera->setTfToBase(tf);
        }

        void LocalMapper::setK(const Eigen::Matrix3d& K) {

            this->camera->setK(K);
        }

        lart_msgs::msg::ConeArray LocalMapper::getCurrentMap() {
            // this method is the map consumer

            std::unique_lock lock(this->mapMutex);

            // wait for the condition variable
            this->mapCond.wait(lock, [&]() { return this->mapReady; });

            // update the flag
            this->mapReady = false;

            return currentMap;
        }
    } // t24e
} // local_mapper