//
// Created by carlostojal on 01-06-2023.
//

#ifndef LOCAL_MAPPING_CORE_CONEDETECTOR_H
#define LOCAL_MAPPING_CORE_CONEDETECTOR_H

#include <local_mapping_ros/cnn/types.h>
#include <opencv2/opencv.hpp>

namespace t24e::local_mapper::cnn {

    enum cone_class_t {
        YELLOW_CONE,
        BLUE_CONE,
        ORANGE_CONE,
        LARGE_ORANGE_CONE,
        CONE_CLASS_COUNT
        // TODO: confirm order with DAMO-YOLO configs
    };

    class ConeDetector {

        protected:
            /*! \brief Is the initialization done? */
            bool initDone = false;

        public:

            /*! \brief Do the necessary initializations. */
            virtual void init() = 0;

            /*! \brief Detect cones on a color image. */
            virtual std::vector<bounding_box_t> detectCones(const cv::Mat& img) = 0;
    };
} // t24e::local_mapper::cnn

#endif //LOCAL_MAPPING_CORE_CONEDETECTOR_H
