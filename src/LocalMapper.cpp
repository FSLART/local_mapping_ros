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

        }

        LocalMapper::~LocalMapper() {
            // explicitly delete the camera pointer, to be sure
            this->camera.reset();

            // explicitly delete the detector instance
            this->detector.reset();
        }

        void LocalMapper::onDepthImage() {

            // this method is basically the map producer

            std::unique_lock lock(this->mapMutex);

            // clear the current map
            this->currentMap.clear();

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

                // add to the map
                std::unique_lock lock(this->mapMutex);
                this->currentMap.push_back(point);
            }

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

        std::vector<geometry_msgs::msg::Point> LocalMapper::getCurrentMap() {
            // this method is the map consumer

            std::unique_lock lock(this->mapMutex);

            // wait for the condition variable
            this->mapCond.wait(lock, [&]() { return this->mapReady; });

            // update the flag
            this->mapReady = false;

            return this->currentMap;
        }
    } // t24e
} // local_mapper