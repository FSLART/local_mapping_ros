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
#include <torch/torch.h>

namespace t24e::local_mapper::post_processing {

    /*! \brief Prediction filtering (post-processing) related to accurate bounding box extractions. */
    class Filtering {

        public:
            Filtering();

            /*! \brief Calculate the entropy for a single row. */
            static float entropyRow(at::Tensor row);

            /*! \brief Calculate the maximum for a single row. */
            static float maxRow(at::Tensor row);

            /*! \brief Filter the predictions based on the entropy of the prediction. 
                \param predictions The predictions scores Torch tensor.
                \param entThreshold The maximum entropy threshold.
                \param scoreThreshold The minimum score threshold.
                \return A pair containing filtered predictions tensor with the allowed entropy and the number of predictions.
            */
            static std::pair<at::Tensor,size_t> filter(at::Tensor predictions,
            float entThreshold, float scoreThreshold);

            /*! \brief Calculate Intersection over Union for a pair of boxes 
                \param box1 The first bounding box.
                \param box2 The second bounding box.
                \return The intersection over union.
            */
            static float calculateIoU(cnn::bounding_box_t box1, cnn::bounding_box_t box2);

            /*! \brief Filter predictions using non-maximum supression intersection over union.
                \param predictions The predictions scores Torch tensor.
                \param scoreThreshold The minimun score threshold.
                \param IoUThreshold The intersection over union threshold.
                \param boundingBoxes The bounding boxes Torch tensor.
            */
            static std::vector<cnn::bounding_box_t> nmsIoU(at::Tensor predictions, float entThreshold, float scoreThreshold, float IoUThreshold,
            at::Tensor boundingBoxes);
    };

};

#endif //LOCAL_MAPPING_CORE_FILTERING_H