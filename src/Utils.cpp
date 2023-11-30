//
// Created by carlostojal on 30-11-2023.
//

#include <local_mapping_ros/Utils.h>

using namespace t24e::local_mapper;

std::vector<std::uint8_t> Utils::getColorByLabel(std::int16_t label) {

    cnn::cone_class_t l = (cnn::cone_class_t) label;

    switch(l) {

        case cnn::cone_class_t::YELLOW_CONE:
            return {255, 255, 0};
        case cnn::cone_class_t::BLUE_CONE:
            return {0, 0, 255};
        case cnn::cone_class_t::ORANGE_CONE:
        case cnn::cone_class_t::LARGE_ORANGE_CONE:
            return {255, 80, 0};
        default:
            return {0, 255, 0};
    }

}