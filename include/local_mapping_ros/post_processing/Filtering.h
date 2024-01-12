//
// Created by carlostojal on 12-01-2024.
//

#ifndef LOCAL_MAPPING_CORE_FILTERING_H
#define LOCAL_MAPPING_CORE_FILTERING_H

#include <local_mapping_ros/cnn/types.h>
#include <vector>
#include <torch/torch.h>

namespace t24e::local_mapper::post_processing {

    /*! \brief Prediction filtering (post-processing) related to accurate bounding box extractions. */
    class Filtering {

        public:
            Filtering();

            /*! \brief Calculate the entropy for a single row. */
            static float entropyRow(torch::Tensor& row);

            /*! \brief Filter the predictions based on the entropy of the prediction. 
                \param predictions The predictions scores Torch tensor.
                \param threshold The maximum entropy threshold.
                \return Filtering predictions tensor with the allowed entropy.
            */
            static torch::Tensor filterByEntropy(torch::Tensor& predictions,
            float threshold);

            /*! \brief Filter predictions using non-maximum supression intersection over union.
                \param predictions The predictions scores Torch tensor.
                \param scoreThreshold The minimun score threshold.
                \param IoUThreshold The intersection over union threshold.
                \param boundingBoxes The bounding boxes Torch tensor.
            */
            static std::vector<cnn::bounding_box_t> nmsIoU(torch::Tensor& predictions, float scoreThreshold, float IoUThreshold,
            torch::Tensor& boundingBoxes);
    };

};

#endif //LOCAL_MAPPING_CORE_FILTERING_H