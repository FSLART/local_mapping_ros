//
// Created by carlostojal on 01-06-2023.
//

#include <local_mapping_ros/vision/RGBCamera.h>
#include <local_mapping_ros/vision/Utils.h>

namespace t24e::local_mapper::vision {

    StampedImage RGBCamera::getLastColorImage() {
        if(!this->colorImageSet)
            throw std::runtime_error("No color image was ever set!");
        std::unique_lock lock(this->colorImageMutex);
        // wait for an image to be available without deadlocking
        this->colorImageCond.wait_for(lock, std::chrono::milliseconds(RGB_CAMERA_IMAGE_MAX_WAIT));
        return lastColorImage;
    }

    void RGBCamera::captureColorImage(const cv::Mat &img) {
        std::unique_lock lock(this->colorImageMutex);
        this->lastColorImage = t24e::local_mapper::vision::Utils::make_stamped_image(img);
        this->colorImageSet = true;
        this->colorImageCond.notify_one(); // notify anyone waiting for an image
    }

    Eigen::Affine3d RGBCamera::getTfToBase() {
        if(!this->tfToBaseSet)
            throw std::runtime_error("No camera extrinsic transform was ever set!");
        std::unique_lock lock(this->tfMutex);
        this->tfCond.wait_for(lock, std::chrono::milliseconds(RGB_CAMERA_TF_MAX_WAIT));
        return this->tfToBase;
    }

    Eigen::Matrix<double,3,4> RGBCamera::getTfToBaseAsRt() {
        Eigen::Affine3d tf = this->getTfToBase();
        Eigen::Matrix3d rotation = tf.rotation();
        Eigen::Vector3d translation = tf.translation();

        Eigen::Matrix<double,3,4> m;
        m.block<3,3>(0,0) = rotation;
        m.block<3,1>(0,3) = translation;

        return m;
    }

    Eigen::Matrix3d RGBCamera::getR() {
        Eigen::Affine3d tf = this->getTfToBase();
        return tf.rotation();
    }

    Eigen::Vector3d RGBCamera::getT() {
        Eigen::Affine3d tf = this->getTfToBase();
        return tf.translation();
    }

    void RGBCamera::setTfToBase(const Eigen::Affine3d &tf) {
        std::unique_lock lock(this->tfMutex);
        this->tfToBase = tf;
        this->tfToBaseSet = true;
        this->tfCond.notify_one();
    }

    bool RGBCamera::hasColorImage() const {
        return this->colorImageSet;
    }

    bool RGBCamera::isTfDefined() const {
        return this->tfToBaseSet;
    }

    Eigen::Matrix3d RGBCamera::getK() {
        if(!this->kSet)
            throw std::runtime_error("No camera intrinsic matrix was ever set!");
        std::unique_lock lock(this->kMutex);
        this->kCond.wait_for(lock, std::chrono::milliseconds(RGB_CAMERA_K_MAX_WAIT));
        return this->K;
    }

    void RGBCamera::setK(const Eigen::Matrix3d &k) {
        std::unique_lock lock(this->kMutex);
        this->K = k;
        this->kSet = true;
        this->kCond.notify_one();
    }

    bool RGBCamera::isKSet() const {
        return this->kSet;
    }
} // vision