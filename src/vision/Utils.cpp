//
// Created by carlostojal on 01-06-2023.
//

#include "local_mapping_ros/vision/Utils.h"

namespace t24e::local_mapper::vision {
    t24e::local_mapper::vision::StampedImage Utils::make_stamped_image(const cv::Mat &img) {

        t24e::local_mapper::vision::StampedImage result;

        result.first = std::chrono::steady_clock::now();
        result.second = img;

        return result;
    }

} // t24e::local_mapper::vision
