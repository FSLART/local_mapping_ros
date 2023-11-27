//
// Created by carlostojal on 02-06-2023.
//

#ifndef LOCAL_MAPPING_CORE_CNN_TYPES_H
#define LOCAL_MAPPING_CORE_CNN_TYPES_H

#include <utility>
#include <cstdint>

namespace t24e::local_mapper::cnn {

    struct bounding_box_t {
        // the first pair is the origin. the second is width and height
        std::pair<std::pair<int,int>,std::pair<int,int>> box;
        uint32_t label;
    };
}

#endif //LOCAL_MAPPING_CORE_CNN_TYPES_H
