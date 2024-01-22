//
// Created by carlostojal on 01-06-2023.
//

#include <local_mapping_ros/vision/ReconstructionFromDepth.h>

namespace t24e::local_mapper::vision {
    geometry_msgs::msg::Point ReconstructionFromDepth::deprojectPixelToPoint(RGBDCamera &cam, Eigen::Vector3d pixel) {

        Eigen::Vector3d point;

        // get x and y coordinates
        double point_x = (pixel[0] - cam.getK().coeff(0, 2)) / cam.getK().coeff(0, 0);
        double point_y = (pixel[1] - cam.getK().coeff(1, 2)) / cam.getK().coeff(1, 1);
        double point_z = 1;

        // assign the coordinates to the eigen vector
        point << point_x, point_y, point_z;

        // normalize the point to the distance (euclidean norm)
        point = point / point.norm() * pixel[2];

        geometry_msgs::msg::Point point_msg;

        point_msg.x = point.x();
        point_msg.y = point.y();
        point_msg.z = point.z();

        return point_msg;
    }
} // t24e::local_mapper::vision