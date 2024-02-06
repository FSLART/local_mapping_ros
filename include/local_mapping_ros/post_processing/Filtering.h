//
// Created by carlostojal on 12-01-2024.
//

#ifndef LOCAL_MAPPING_CORE_FILTERING_H
#define LOCAL_MAPPING_CORE_FILTERING_H

#include <local_mapping_ros/cnn/types.h>
#include <local_mapping_ros/post_processing/ThreadPool.h>
#include <vector>
#include <mutex>
#include <algorithm>

namespace t24e::local_mapper::post_processing {

    /*! \brief Prediction filtering (post-processing) related to accurate bounding box extractions. */
    class Filtering {

        public:
            Filtering();
    };

};

#endif //LOCAL_MAPPING_CORE_FILTERING_H