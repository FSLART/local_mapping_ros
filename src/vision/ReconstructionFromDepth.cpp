//
// Created by carlostojal on 01-06-2023.
//

#include <local_mapping_ros/vision/ReconstructionFromDepth.h>

namespace t24e {
    namespace local_mapper {
        namespace vision {
            geometry_msgs::msg::Point ReconstructionFromDepth::deprojectPixelToPoint(RGBDCamera &cam, Eigen::Vector3d pixel) {

                geometry_msgs::msg::Point point;

                point.x = (pixel.x() - cam.getK()(0,2)) * pixel.z() / cam.getK()(0,0);
                point.y = (pixel.y() - cam.getK()(1,2)) * pixel.z() / cam.getK()(1,1);
                point.z = pixel.z();

                return point;
            }
        } // t24e
    } // local_mapper
} // vision