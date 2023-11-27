//
// Created by carlostojal on 01-06-2023.
//

#ifndef LOCAL_MAPPING_CORE_RGBDCAMERA_H
#define LOCAL_MAPPING_CORE_RGBDCAMERA_H

#include <local_mapping_ros/vision/RGBCamera.h>
#include <local_mapping_ros/vision/Utils.h>

#define RGBD_CAMERA_DEPTH_MAX_WAIT 200

namespace t24e::local_mapper::vision {

    /*! \brief This class represents any RGB-D camera attached to the car.
     *
     * Besides the functionality offered by the RGB camera class, it keeps the latest depth image.
     */
    class RGBDCamera : public RGBCamera {

        private:
            /*! \brief The latest depth image captured by the camera. */
            StampedImage lastDepthImage;

            /*! \brief Was a depth image set? */
            bool depthImageSet = false;

            /*! \brief Mutex to protect access to the depth image. */
            std::mutex depthImageMutex;

            /*! \brief Condition variable to access the depth image. */
            std::condition_variable depthImageCond;

            // TODO: detect cones and de-project automatically on new images

        public:
            StampedImage getLastDepthImage();
            void captureDepthImage(const cv::Mat& img);

            bool hasDepthImage() const;

            const std::string TYPE = "RGBD";
    };

} // t24e

#endif //LOCAL_MAPPING_CORE_RGBDCAMERA_H
