//
// Created by carlostojal on 01-06-2023.
//

#ifndef LOCAL_MAPPING_CORE_UTILS_H
#define LOCAL_MAPPING_CORE_UTILS_H

#include <local_mapping_ros/vision/types.h>
#include <local_mapping_ros/cnn/types.h>
#include <chrono>

namespace t24e::local_mapper::vision {

    class Utils {
        public:
            /*! \brief Put the current timestamp on an image.
             *
             * \param img The input OpenCV image to apply a timestamp to.
             * */
            static t24e::local_mapper::vision::StampedImage make_stamped_image(const cv::Mat& img);

    };
} // t24e::local_mapper::vision

#endif //LOCAL_MAPPING_CORE_UTILS_H
