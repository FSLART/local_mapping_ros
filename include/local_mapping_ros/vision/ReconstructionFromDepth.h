//
// Created by carlostojal on 01-06-2023.
//

#ifndef LOCAL_MAPPING_CORE_RECONSTRUCTIONFROMDEPTH_H
#define LOCAL_MAPPING_CORE_RECONSTRUCTIONFROMDEPTH_H

#include <local_mapping_ros/vision/RGBDCamera.h>
#include "geometry_msgs/msg/point.hpp"
#include <utility>
#include <eigen3/Eigen/Dense>

namespace t24e {
    namespace local_mapper {
        namespace vision {

            class ReconstructionFromDepth {
                public:
                    static geometry_msgs::msg::Point deprojectPixelToPoint(local_mapper::vision::RGBDCamera& cam,
                                                               Eigen::Vector3d pixel);
            };

        } // t24e
    } // local_mapper
} // vision

#endif //LOCAL_MAPPING_CORE_RECONSTRUCTIONFROMDEPTH_H
