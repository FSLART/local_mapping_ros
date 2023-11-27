//
// Created by carlostojal on 01-06-2023.
//

#ifndef LOCAL_MAPPING_CORE_VISION_TYPES_H
#define LOCAL_MAPPING_CORE_VISION_TYPES_H

#include <utility>
#include <opencv2/opencv.hpp>
#include <chrono>

namespace t24e::local_mapper::vision {

    /*! \brief A pair of a timestamp with an image. */
    typedef std::pair<std::chrono::time_point<std::chrono::steady_clock>, cv::Mat> StampedImage;

}

#endif //LOCAL_MAPPING_CORE_VISION_TYPES_H
