//
// Created by carlostojal on 01-06-2023.
//

#include "local_mapping_ros/cnn/DAMO.h"
#include <local_mapping_ros/vision/Utils.h>

namespace t24e::local_mapper::cnn {

    DAMO::DAMO(std::string& modelPath) {

        this->modelPath = modelPath;
        this->modelPathSet = true;

        #ifdef WITH_CUDA
            this->device = torch::Device(torch::kCUDA);
            this->validateDevice();
        #else
            this->device = torch::Device(torch::kCPU);
        #endif
    }

    void DAMO::validateDevice() {

        if(this->device == torch::Device(torch::kCUDA)) {
            if(!torch::cuda::is_available()) {
                std::cerr << "CUDA is not available!" << std::endl;
                throw std::runtime_error("CUDA is not available!");
            }
        }
    }

    std::vector<bounding_box_t> DAMO::detectCones(cv::Mat img) {

        // resize the image to inference dimensions
        cv::Mat resizedImg;
        cv::resize(img, resizedImg, cv::Size(DETECTOR_WIDTH, DETECTOR_HEIGHT));

        std::cout << this->device.type() << std::endl;

        // convert the opencv image to a tensor
        at::Tensor tensorImage = torch::from_blob(resizedImg.data, {1, 3, resizedImg.rows, resizedImg.cols}, at::kFloat);
        std::cout << "Tensor shape: " << tensorImage.sizes() << std::endl;

        // check the device is available
        this->validateDevice();

        try {
            // move the tensor to the intended device
            tensorImage = tensorImage.to(this->device);
        } catch(const std::exception& e) {
            std::cerr << "Error moving the tensor to the device: " << e.what() << std::endl;
            throw std::runtime_error("Error moving the tensor to the device!");
        }

        // convert to a vector
        std::vector<torch::jit::IValue> input;
        input.emplace_back(tensorImage);

        // execute the model
        auto inference = this->torchModule.forward(input);

        auto outputs = inference.toTuple();

        /*
            tensors:
            [0]: [1, 8400, 5] -> class probabilities
            [1]: [1, 8400, 4] -> box origin and dimensions
        */

        for(size_t i = 0; i < outputs->elements().size(); i++) {
            torch::Tensor elem = outputs->elements()[i].toTensor();

            std::cout << "Elem " << i << ": " << elem.sizes() << std::endl;
        }

        std::vector<bounding_box_t> bounding_boxes;

        /*
        at::Tensor outputs = inference.toTensor();

        // extract the bounding boxes
        for(int i = 0; i < outputs.size(0); i++) {
            bounding_box_t box;
            box.box.first.first = outputs[i][0].item().toInt();
            box.box.first.second = outputs[i][1].item().toInt();

            box.box.second.first = outputs[i][2].item().toInt();
            box.box.second.second = outputs[i][3].item().toInt();

            box.label = outputs[i][4].item().toInt();
            bounding_boxes.push_back(box);
        }*/

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
            this->validateDevice();

            this->torchModule = torch::jit::load(this->modelPath, this->device);
            
            this->torchModule.eval(); // set the model to evaluation mode

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
