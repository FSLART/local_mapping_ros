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

    float Filtering::maxRow(torch::Tensor& row) {
        // calculate the entropy of the row
        float max = 0;
        for(int i = 0; i < row.size(0); i++) {
            float p = row[i].item<float>();
            if(p > max) {
                max = p;
            }
        }
        return max;
    }

    std::pair<torch::Tensor,size_t> torch::Tensor Filtering::filter(torch::Tensor& predictions, float entThreshold, float scoreThreshold){
        // filtered row counter
        size_t numRows = 0;

        // initialize the filtered predictions tensor with the same size as the predictions tensor filled with zeros
        torch::Tensor filteredPredictions = torch::zeros_like(predictions);

        // TODO: implement thread pooling
        for(int i = 0; i < predictions.size(0); i++) {

            torch::Tensor row = predictions[i];

            // calculate the entropy and maximum of the row
            float entropy = entropyRow(row);
            float max = maxRow(row);

            // verify filter conditions 
            if(entropy < entThreshold && max > scoreThreshold) {
                filteredPredictions[numRows++] = row;
            }
        }
        return std::make_pair(filteredPredictions, numRows);
    }

    float Filtering::calculateIoU(cnn::bounding_box_t& box1, cnn::bounding_box_t& box2) {
        
        // get coordinates of intersection rectangle
        int xA = std::max(box1.box.first.first, box2.box.first.first);
        int yA = std::max(box1.box.first.second, box2.box.first.second);
        int xB = std::min(box1.box.first.first + box1.box.second.first, box2.box.first.first + box2.box.second.first);
        int yB = std::min(box1.box.first.second + box1.box.second.second, box2.box.first.second + box2.box.second.second);


        // compute the area of intersection rectangle
        int interArea = std::max(0, xB - xA + 1) * std::max(0, yB - yA);

        // compute the area of the union
        int area1 = box1.box.second.first * box1.box.second.second;
        int area2 = box2.box.second.first * box2.box.second.second;
        int unionArea = area1 + area2 - interArea;

        // compute the intersection over union
        float iou = static_cast<float>(interArea) / static_cast<float>unionArea;

        return iou;
    }

    std::vector<cnn::bounding_box_t> Filtering::nmsIoU(torch::Tensor& predictions, float scoreThreshold, float IoUThreshold,
    torch::Tensor& boundingBoxes, bool useEntropy) {

        // TODO: perform non-maximum supression
        std::vector<cnn::bounding_box_t> boundingBoxes;
        
        return boundingBoxes;
    }
};