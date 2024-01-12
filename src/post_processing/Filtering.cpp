//
// Created by carlostojal on 12-01-2024.
//

#include <local_mapping_ros/post_processing/Filtering.h>

namespace t24e::local_mapper::post_processing {

    float Filtering::entropyRow(torch::Tensor& row) {
        // calculate the entropy of the row
        float entropy = 0;
        for(int i = 0; i < row.size(0); i++) {
            float p = row[i].item<float>();
            entropy += p * std::log(p);
        }
        return -entropy;
    }

};