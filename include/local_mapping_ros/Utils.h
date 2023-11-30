//
// Created by carlostojal on 30-11-2023.
//

#ifndef LOCAL_MAPPING_ROS_UTILS_H
#define LOCAL_MAPPING_ROS_UTILS_H

#include <local_mapping_ros/cnn/ConeDetector.h>
#include <vector>
#include <cstdint>

namespace t24e::local_mapper {

    class Utils {

        public:
            /*! \brief Get the color by the cone label. */
            static std::vector<std::uint8_t> getColorByLabel(std::int16_t label);

    };
}

#endif