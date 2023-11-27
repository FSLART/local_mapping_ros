//
// Created by carlostojal on 01-06-2023.
//

#ifndef LOCAL_MAPPING_CORE_DAMO_H
#define LOCAL_MAPPING_CORE_DAMO_H

#include <local_mapping_ros/cnn/ConeDetector.h>
#include <torch/script.h>

namespace t24e::local_mapper::cnn {

    class DAMO : private ConeDetector {

        private:
            /*! \brief The TorchScript model. */
            torch::jit::script::Module torchModule;

            /*! \brief The TorchScript model path to load. */
            std::string modelPath;

            /*! \brief Was the model path set? */
            bool modelPathSet = false;

        public:
            DAMO(std::string& modelPath);

            void init() override;

            std::vector<bounding_box_t> detectCones(const cv::Mat& img) override;

            /*! \brief TorchScript model path getter. */
            std::string getModelPath() const;

            /*! \brief TorchScript model path setter. */
            void setModelPath(const std::string& path);


    };
} // t24e::local_mapper::cnn

#endif //LOCAL_MAPPING_CORE_DAMO_H
