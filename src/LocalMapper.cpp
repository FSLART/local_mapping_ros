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