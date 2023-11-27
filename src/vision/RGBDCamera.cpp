//
// Created by carlostojal on 01-06-2023.
//

#include <local_mapping_ros/vision/RGBDCamera.h>

namespace t24e::local_mapper::vision {

    StampedImage RGBDCamera::getLastDepthImage() {
        if(!this->depthImageSet)
            throw std::runtime_error("No depth image was ever set!");
        std::unique_lock lock(this->depthImageMutex);
        this->depthImageCond.wait_for(lock, std::chrono::milliseconds(RGBD_CAMERA_DEPTH_MAX_WAIT));
        return this->lastDepthImage;
    }

    void RGBDCamera::captureDepthImage(const cv::Mat &img) {
        std::unique_lock lock(this->depthImageMutex);
        this->lastDepthImage = local_mapper::vision::Utils::make_stamped_image(img);
        this->depthImageSet = true;
        this->depthImageCond.notify_one();
    }

    bool RGBDCamera::hasDepthImage() const {
        return this->depthImageSet;
    }

} // RGBDCamera