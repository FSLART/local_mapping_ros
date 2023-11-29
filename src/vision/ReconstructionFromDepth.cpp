//
// Created by carlostojal on 01-06-2023.
//

#include <local_mapping_ros/vision/ReconstructionFromDepth.h>

namespace t24e::local_mapper::vision {
    geometry_msgs::msg::Point ReconstructionFromDepth::deprojectPixelToPoint(RGBDCamera &cam, Eigen::Vector3d pixel) {

        Eigen::Vector3d point;

        point = (cam.getR().inverse() * cam.getK().inverse() * pixel) - cam.getT();

        geometry_msgs::msg::Point point_msg;

        point_msg.x = point.x();
        point_msg.y = point.y();
        point_msg.z = point.z();

        return point;
    }
} // t24e::local_mapper::vision