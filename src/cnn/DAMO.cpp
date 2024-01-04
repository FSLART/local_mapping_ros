//
// Created by carlostojal on 01-06-2023.
//

#include "local_mapping_ros/cnn/DAMO.h"
#include <local_mapping_ros/vision/Utils.h>

namespace t24e::local_mapper::cnn {

    DAMO::DAMO(std::string& modelPath) {

        this->modelPath = modelPath;
        this->modelPathSet = true;
    }

    std::vector<bounding_box_t> DAMO::detectCones(const cv::Mat& img) {

        // convert the opencv image to a tensor
        at::Tensor tensorImage = torch::from_blob(img.data, {1, 3, img.rows, img.cols}, at::kFloat);
        // move the tensor to the GPU
        tensorImage = tensorImage.to(at::kCUDA);

        // convert to a vector
        std::vector<torch::jit::IValue> input;
        input.emplace_back(tensorImage);

        // execute the model
        torch::Tensor outputs = this->torchModule.forward(input).toTensor();

        // extract the bounding boxes
        std::vector<bounding_box_t> bounding_boxes;
        for(int i = 0; i < outputs.size(0); i++) {
            bounding_box_t box;
            box.box.first.first = outputs[i][0].item().toInt();
            box.box.first.second = outputs[i][1].item().toInt();

            box.box.second.first = outputs[i][2].item().toInt();
            box.box.second.second = outputs[i][3].item().toInt();

            box.label = outputs[i][4].item().toInt();
            bounding_boxes.push_back(box);
        }

        // return a vector of bounding boxes
        return bounding_boxes;
    }

    void DAMO::init() {

        // verify the model path has been set
        if(!this->modelPathSet)
            throw std::runtime_error("Model path must be set before initialization!");

        // load the torchscript model
        try {
            std::cout << "Loading the TorchScript module at " << this->modelPath << std::endl;
            this->torchModule = torch::jit::load(this->modelPath);
        } catch(const c10::Error& e) {
            std::cerr << "Error loading the TorchScript module: " << e.what() << std::endl;
            throw std::runtime_error("Error loading the TorchScript module!");
        }

        std::cout << "TorchScript module loaded successfully!" << std::endl;

        this->initDone = true;
    }

    std::string DAMO::getModelPath() const {
        // verify the model path has been set
        if(!this->modelPathSet) {
            throw std::runtime_error("The model path was not set!");
        }

        return this->modelPath;
    }

    void DAMO::setModelPath(const std::string& path) {
        this->modelPathSet = true;
        this->modelPath = path;
    }

} // t24e::local_mapper::cnn
