//
// Created by carlostojal on 01-06-2023.
//

#ifndef LOCAL_MAPPING_CORE_DAMO_H
#define LOCAL_MAPPING_CORE_DAMO_H

#include <local_mapping_ros/cnn/ConeDetector.h>
#include <local_mapping_ros/vision/Utils.h>
#include <local_mapping_ros/post_processing/ThreadPool.h>
#include <local_mapping_ros/post_processing/Filtering.h>
#include <torch/torch.h>
#include <torch/script.h>
#include <thread>
#include <set>
#include <mutex>
#include <cmath>

#define MAX_ENTROPY_THRESHOLD 0.3
#define MIN_SCORE_THRESHOLD 0.5
#define IOU_THRESHOLD 0.5

namespace t24e::local_mapper::cnn {

    class DAMO : private ConeDetector {

        private:

            /*! \brief The ONNX session. */
            Ort::Session session;

            /*! \brief The ONNX model path to load. */
            std::string modelPath;

            /*! \brief Was the model path set? */
            bool modelPathSet = false;

            std::unique_ptr<ThreadPool> threadPool;

        public:
            DAMO(std::string& modelPath);

            void init() override;

            std::vector<bounding_box_t> detectCones(cv::Mat img) override;

            /*! \brief TorchScript model path getter. */
            std::string getModelPath() const;

            /*! \brief TorchScript model path setter. */
            void setModelPath(const std::string& path);


    };
} // t24e::local_mapper::cnn

#endif //LOCAL_MAPPING_CORE_DAMO_H
